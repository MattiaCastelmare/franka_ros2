"""Acceleration HQPs with hard limits and optional prioritized comfort slack.

The decision variable is the *actuator-induced* acceleration
``u = solve(M, tau - c)``. External disturbances belong in the bounds supplied
by the caller, not in an extra torque compensation term here. By default the
Cartesian action is solved first; the secondary solve preserves its attained
``J @ u`` exactly, up to a checked numerical equality tolerance.
The elastic variant first minimizes relaxation of separate comfort bounds.

OSQP instances retain their matrix sparsity and warm starts between updates.
No result is used without an independent check in the original physical units.
"""
from __future__ import annotations

from dataclasses import dataclass
from time import perf_counter

import numpy as np
import osqp
from scipy import sparse


class QPInfeasibleError(RuntimeError):
    """No verified primary solution is available; never reuse a stale command."""


@dataclass(frozen=True)
class HierarchicalQPResult:
    """Verified command and scalar diagnostics for one controller update."""

    acceleration: np.ndarray
    primary_acceleration: np.ndarray
    primary_status: int
    secondary_status: int
    primary_iterations: int
    secondary_iterations: int
    max_violation: float
    primary_ms: float
    secondary_ms: float
    soft_slack_lower: np.ndarray | None = None
    soft_slack_upper: np.ndarray | None = None
    safety_status: int | None = None
    safety_iterations: int = 0
    safety_ms: float = 0.0


def _positive(value, name):
    array = np.asarray(value, dtype=float)
    if array.ndim != 0 or not np.isfinite(array) or array <= 0.0:
        raise ValueError(f'{name} must be a finite positive scalar')
    return float(array)


def _positive_integer(value, name):
    number = _positive(value, name)
    if isinstance(value, (bool, np.bool_)) or not number.is_integer():
        raise ValueError(f'{name} must be a positive integer')
    return int(number)


def _boolean(value, name):
    if not isinstance(value, (bool, np.bool_)):
        raise ValueError(f'{name} must be a boolean')
    return bool(value)


def _array(value, shape, name):
    array = np.asarray(value, dtype=float)
    if array.shape != shape or not np.all(np.isfinite(array)):
        raise ValueError(f'{name} must have shape {shape} and finite entries')
    return array


class HierarchicalAccelerationQP:
    """Prioritize Cartesian action, then redundant posture/mass/damping.

    Primary problem::

        min 0.5 ||J u - task_target||^2 + 0.5 regularization ||u||^2

    Secondary problem::

        min 0.5 ||u - secondary_target||^2
        s.t. J u = J u_primary

    Both levels impose ``lower <= u <= upper`` and
    ``-torque_limits <= M u + c <= torque_limits`` without slack variables.
    The small primary regularizer makes its unconstrained Hessian positive
    definite; hierarchy is enforced relative to the primary solution attained
    with that regularizer, not an unregularized ideal solution.

    The Cartesian row count is fixed on the first call. A failed secondary
    solve returns that call's verified primary solution. A failed primary
    solve raises :class:`QPInfeasibleError`, leaving fail-stop policy to the
    controller/runner. Neither case returns an earlier call's acceleration.
    """

    def __init__(self, n=7, settings=None):
        self.n = _positive_integer(n, 'n')
        values = {
            'primary_regularization': 1e-8,
            'feasibility_tolerance': 2e-4,
            'equality_tolerance': 2e-5,
            'eps_abs': 1e-6,
            'eps_rel': 1e-6,
            'max_iter': 1000,
            'check_termination': 10,
            'polish': False,
            'adaptive_rho': True,
            'rho': 0.1,
        }
        incoming = dict(settings or {})
        unknown = set(incoming) - set(values)
        if unknown:
            raise ValueError(f'unknown QP settings: {sorted(unknown)}')
        values.update(incoming)
        self.regularization = _positive(values.pop('primary_regularization'),
                                        'primary_regularization')
        self.feasibility_tolerance = _positive(values.pop('feasibility_tolerance'),
                                              'feasibility_tolerance')
        self.equality_tolerance = _positive(values.pop('equality_tolerance'),
                                           'equality_tolerance')
        for key in ('eps_abs', 'eps_rel', 'rho'):
            values[key] = _positive(values[key], key)
        for key in ('max_iter', 'check_termination'):
            values[key] = _positive_integer(values[key], key)
        for key in ('polish', 'adaptive_rho'):
            values[key] = _boolean(values[key], key)
        # OSQP renamed these settings in 1.0; support both installed interfaces.
        if int(osqp.__version__.split('.')[0]) >= 1:
            values['polishing'] = values.pop('polish')
            values['warm_starting'] = True
        else:
            values['warm_start'] = True
        values['verbose'] = False
        self.settings = values
        self._primary = None
        self._secondary = None
        self._task_rows = None
        self._P_pattern = sparse.triu(np.ones((self.n, self.n)), format='csc')
        self._P_rows = self._P_pattern.indices
        self._P_columns = np.repeat(np.arange(self.n), np.diff(self._P_pattern.indptr))

    def _matrix(self, values):
        """Keep explicit zeros so numerical updates never alter CSC sparsity."""
        result = sparse.csc_matrix(np.ones(values.shape))
        result.data[:] = values.ravel(order='F')
        return result

    def _hessian(self, values):
        result = self._P_pattern.copy()
        result.data[:] = values[self._P_rows, self._P_columns]
        return result

    def _run(self, level, P, linear, A, lower, upper):
        started = perf_counter()
        solver = getattr(self, '_' + level)
        if solver is None:
            solver = osqp.OSQP()
            solver.setup(P=self._hessian(P), q=linear, A=self._matrix(A),
                         l=lower, u=upper, **self.settings)
            setattr(self, '_' + level, solver)
        else:
            solver.update(Px=P[self._P_rows, self._P_columns], q=linear,
                          Ax=A.ravel(order='F'), l=lower, u=upper)
        result = solver.solve()
        return result, 1e3 * (perf_counter() - started)

    def _violation(self, acceleration, M, c, lower, upper, torque_limits):
        if acceleration is None:
            return np.inf
        acceleration = np.asarray(acceleration, dtype=float)
        if acceleration.shape != (self.n,) or not np.all(np.isfinite(acceleration)):
            return np.inf
        torque = M @ acceleration + c
        return float(max(0.0, np.max(lower - acceleration),
                         np.max(acceleration - upper),
                         np.max(np.abs(torque) - torque_limits)))

    def solve(self, J, task_target, secondary_target, M, c,
              lower, upper, torque_limits):
        """Return this update's verified hierarchical acceleration.

        ``lower/upper`` constrain actuator acceleration; the caller must shift
        physical acceleration constraints by its external-disturbance estimate
        and tighten them for uncertainty before this call. ``torque_limits``
        constrain commanded gravity-compensated actuator torque in N m.
        """
        J = np.asarray(J, dtype=float)
        if (J.ndim != 2 or J.shape[1] != self.n or J.shape[0] < 1
                or J.shape[0] > self.n or not np.all(np.isfinite(J))):
            raise ValueError(f'J must be a finite task Jacobian with {self.n} columns')
        rows = J.shape[0]
        if self._task_rows is None:
            self._task_rows = rows
        elif rows != self._task_rows:
            raise ValueError('task Jacobian row count cannot change after the first solve')
        task_target = _array(task_target, (rows,), 'task_target')
        secondary_target = _array(secondary_target, (self.n,), 'secondary_target')
        M = _array(M, (self.n, self.n), 'M')
        c = _array(c, (self.n,), 'c')
        lower = _array(lower, (self.n,), 'lower')
        upper = _array(upper, (self.n,), 'upper')
        torque_limits = _array(torque_limits, (self.n,), 'torque_limits')
        if np.any(torque_limits <= 0.0):
            raise ValueError('torque_limits must be positive')
        if np.any(lower > upper):
            joint = int(np.argmax(lower - upper))
            raise QPInfeasibleError(
                f'contradictory acceleration bounds for joint {joint + 1}: '
                f'lower={lower[joint]:.6g} > upper={upper[joint]:.6g}')

        common_A = np.vstack((np.eye(self.n), M))
        common_lower = np.r_[lower, -torque_limits - c]
        common_upper = np.r_[upper, torque_limits - c]
        primary_P = J.T @ J + self.regularization * np.eye(self.n)
        try:
            primary, primary_ms = self._run(
                'primary', primary_P, -J.T @ task_target,
                common_A, common_lower, common_upper)
        except Exception as exc:
            raise QPInfeasibleError(f'primary QP solver failed: {exc}') from exc
        primary_violation = self._violation(primary.x, M, c, lower, upper, torque_limits)
        if (primary.info.status_val not in (1, 2)
                or primary_violation > self.feasibility_tolerance):
            raise QPInfeasibleError(
                f'primary QP has no verified solution: {primary.info.status} '
                f'(status={primary.info.status_val}, iterations={primary.info.iter}, '
                f'hard violation={primary_violation:.6g})')
        first = np.asarray(primary.x, dtype=float).copy()
        attained = J @ first
        secondary_A = np.vstack((common_A, J))
        secondary_status, secondary_iterations = -1, 0
        selected = first
        violation = primary_violation
        secondary_started = perf_counter()
        try:
            secondary, secondary_ms = self._run(
                'secondary', np.eye(self.n), -secondary_target, secondary_A,
                np.r_[common_lower, attained], np.r_[common_upper, attained])
            secondary_status = int(secondary.info.status_val)
            secondary_iterations = int(secondary.info.iter)
            second_violation = self._violation(
                secondary.x, M, c, lower, upper, torque_limits)
            equality_error = (float(np.max(np.abs(J @ secondary.x - attained)))
                              if np.isfinite(second_violation) else np.inf)
            if (secondary_status in (1, 2)
                    and second_violation <= self.feasibility_tolerance
                    and equality_error <= self.equality_tolerance):
                selected = np.asarray(secondary.x, dtype=float).copy()
                violation = max(second_violation, equality_error)
            else:
                # A solved status with a failed independent check is distinct
                # from an accepted solve, even if OSQP's own tolerance passed.
                if secondary_status in (1, 2):
                    secondary_status = -2
        except Exception:
            # The verified primary solution is still from this same update.
            # A negative secondary status reports this degraded hierarchy.
            secondary_ms = 1e3 * (perf_counter() - secondary_started)

        return HierarchicalQPResult(
            acceleration=selected.copy(), primary_acceleration=first,
            primary_status=int(primary.info.status_val),
            secondary_status=secondary_status,
            primary_iterations=int(primary.info.iter),
            secondary_iterations=secondary_iterations,
            max_violation=float(violation), primary_ms=primary_ms,
            secondary_ms=secondary_ms)


class ElasticHierarchicalAccelerationQP(HierarchicalAccelerationQP):
    """Minimize comfort-limit relaxation before tracking and redundant motion.

    The first level retains all supplied hard joint/torque constraints while
    minimizing squared, dimensionless lower/upper slacks. Its Hessian has no
    acceleration regularizer: actuator acceleration is free wherever the same
    optimal slack vector can be attained. Groups of soft bounds each constrain
    the joint acceleration directly; their scale converts physical units to
    dimensionless violations.

    The verified slack vector is then frozen, up to ``equality_tolerance`` in
    physical units, before the existing tracking/redundancy hierarchy runs.
    Failure of that tracking solve returns the freshly verified first-level
    command. Failure to obtain a verified hard-feasible first level raises
    ``QPInfeasibleError``. No command from an earlier update is reused.
    """

    def __init__(self, n=7, settings=None):
        super().__init__(n, settings)
        self._safety = None
        self._soft_groups = 3
        self._safety_A = None
        self._safety_lower = None
        self._safety_upper = None

    def _run_safety(self, M, c, lower, upper, torque_limits,
                    soft_lower, soft_upper, soft_scale):
        """Solve the sparse slack-only QP, preserving its pattern between calls."""
        started = perf_counter()
        n, count = self.n, soft_lower.size
        size = n + 2 * count
        # Optimize v = u / acceleration_scale, and normalize torque rows by
        # their limits. Thus soft slacks and actuator variables have comparable
        # numerical scales without changing the physical feasible set.
        acceleration_scale = soft_scale[0]
        normalized_M = M * acceleration_scale[None, :] / torque_limits[:, None]
        soft_coefficients = (acceleration_scale / soft_scale).ravel()
        if self._safety_A is None:
            joint = np.arange(count) % n
            index = np.arange(count)
            # Rows: hard acceleration, torque, lower/upper soft bounds, z >= 0.
            rows = np.r_[np.arange(n),
                         n + np.tile(np.arange(n), n),
                         2 * n + index, 2 * n + index,
                         2 * n + count + index, 2 * n + count + index,
                         2 * n + 2 * count + np.arange(2 * count)]
            columns = np.r_[np.arange(n), np.repeat(np.arange(n), n),
                            joint, n + index, joint, n + count + index,
                            n + np.arange(2 * count)]
            values = np.r_[np.ones(n), normalized_M.ravel(order='F'),
                           soft_coefficients, np.ones(count),
                           soft_coefficients, -np.ones(count),
                           np.ones(2 * count)]
            A = sparse.csc_matrix((values, (rows, columns)),
                                  shape=(2 * n + 4 * count, size))
            self._safety_A = A
            data_columns = np.repeat(np.arange(size), np.diff(A.indptr))
            actuator = data_columns < n
            self._safety_M_data = np.flatnonzero(
                actuator & (A.indices >= n) & (A.indices < 2 * n))
            self._safety_soft_data = np.flatnonzero(
                actuator & (A.indices >= 2 * n) & (A.indices < 2 * n + 2 * count))
            self._safety_soft_rows = (A.indices[self._safety_soft_data] - 2 * n) % count
            self._safety_lower = np.full(A.shape[0], -np.inf)
            self._safety_lower[2 * n + 2 * count:] = 0.0
            self._safety_upper = np.full(A.shape[0], np.inf)
        else:
            A = self._safety_A
            # Only numerical coefficients change; CSC order and explicit zeros
            # remain exactly those of the first sparse construction.
            A.data[self._safety_M_data] = normalized_M.ravel(order='F')
            A.data[self._safety_soft_data] = soft_coefficients[self._safety_soft_rows]
        lo, hi = self._safety_lower, self._safety_upper
        lo[:n] = lower / acceleration_scale
        lo[n:2 * n] = -1.0 - c / torque_limits
        lo[2 * n:2 * n + count] = (soft_lower / soft_scale).ravel()
        hi[:n] = upper / acceleration_scale
        hi[n:2 * n] = 1.0 - c / torque_limits
        hi[2 * n + count:2 * n + 2 * count] = (soft_upper / soft_scale).ravel()
        if self._safety is None:
            # Explicit zero acceleration weights make this a lexicographic
            # slack minimum, rather than a weighted trade-off with motion.
            P = sparse.diags(np.r_[np.zeros(n), np.ones(2 * count)], format='csc')
            self._safety = osqp.OSQP()
            self._safety.setup(P=P, q=np.zeros(size), A=A, l=lo, u=hi,
                               **self.settings)
        else:
            self._safety.update(Ax=A.data, l=lo, u=hi)
        result = self._safety.solve()
        return result, 1e3 * (perf_counter() - started)

    def solve(self, J, task_target, secondary_target, M, c,
              lower, upper, torque_limits, *, soft_lower, soft_upper, soft_scale):
        """Keep physical bounds hard and minimize relaxation of three soft groups.

        Soft arrays have shape ``(3, n)`` and are already shifted for the
        disturbance estimate and tightened for its uncertainty. Their order is
        supplied by the caller (acceleration, position and velocity preferences
        in the controller). Contradictory soft lower/upper bounds are allowed:
        resolving that contradiction is the purpose of the slack variables.
        """
        n, shape = self.n, (self._soft_groups, self.n)
        # Validate task inputs before computing a fallback: invalid task data
        # must not be mistaken for a numerical tracking-solver failure.
        J = np.asarray(J, dtype=float)
        if (J.ndim != 2 or J.shape[1] != n or not 0 < J.shape[0] <= n
                or not np.all(np.isfinite(J))):
            raise ValueError(f'J must be a finite task Jacobian with {n} columns')
        if self._task_rows is not None and J.shape[0] != self._task_rows:
            raise ValueError('task Jacobian row count cannot change after the first solve')
        self._task_rows = J.shape[0]
        task_target = _array(task_target, (J.shape[0],), 'task_target')
        secondary_target = _array(secondary_target, (n,), 'secondary_target')
        M, c = _array(M, (n, n), 'M'), _array(c, (n,), 'c')
        lower, upper = _array(lower, (n,), 'lower'), _array(upper, (n,), 'upper')
        torque_limits = _array(torque_limits, (n,), 'torque_limits')
        soft_lower = _array(soft_lower, shape, 'soft_lower')
        soft_upper = _array(soft_upper, shape, 'soft_upper')
        soft_scale = _array(soft_scale, shape, 'soft_scale')
        if np.any(soft_scale <= 0.0) or np.any(torque_limits <= 0.0):
            raise ValueError('soft scales and torque limits must be positive')
        if np.any(lower > upper):
            joint = int(np.argmax(lower - upper))
            raise QPInfeasibleError(
                f'contradictory hard acceleration bounds for joint {joint + 1}: '
                f'lower={lower[joint]:.6g} > upper={upper[joint]:.6g}')
        try:
            safety, safety_ms = self._run_safety(
                M, c, lower, upper, torque_limits, soft_lower, soft_upper, soft_scale)
        except Exception as exc:
            raise QPInfeasibleError(f'hard-constraint/slack QP solver failed: {exc}') from exc
        count = soft_lower.size
        if safety.x is None or np.asarray(safety.x).shape != (n + 2 * count,):
            raise QPInfeasibleError('hard-constraint/slack QP returned no complete command')
        values = np.asarray(safety.x, dtype=float)
        first = values[:n] * soft_scale[0]
        slack_lower = values[n:n + count].reshape(shape) * soft_scale
        slack_upper = values[n + count:].reshape(shape) * soft_scale
        hard_error = self._violation(first, M, c, lower, upper, torque_limits)
        soft_error = float(max(0.0,
            np.max(soft_lower - first - slack_lower),
            np.max(first - soft_upper - slack_upper),
            np.max(-slack_lower), np.max(-slack_upper)))
        if (safety.info.status_val not in (1, 2) or not np.all(np.isfinite(values))
                or max(hard_error, soft_error) > self.feasibility_tolerance):
            raise QPInfeasibleError(
                f'hard-constraint/slack QP has no verified solution: {safety.info.status} '
                f'(status={safety.info.status_val}, iterations={safety.info.iter}, '
                f'hard violation={hard_error:.6g}, slack violation={soft_error:.6g})')

        # Include verified numerical residuals so that the fresh first-level
        # command remains a witness for the frozen slack box. This adjustment
        # is bounded by the independent physical-unit check above.
        slack_lower = np.maximum.reduce((np.zeros(shape), slack_lower,
                                          soft_lower - first))
        slack_upper = np.maximum.reduce((np.zeros(shape), slack_upper,
                                          first - soft_upper))
        tolerance = self.equality_tolerance
        frozen_lower = np.maximum(lower, np.max(soft_lower - slack_lower - tolerance, axis=0))
        frozen_upper = np.minimum(upper, np.min(soft_upper + slack_upper + tolerance, axis=0))
        tracking_started = perf_counter()
        try:
            task = super().solve(J, task_target, secondary_target, M, c,
                                 frozen_lower, frozen_upper, torque_limits)
        except QPInfeasibleError:
            # Safety/slack optimization succeeded for THIS measurement. A
            # numerical task failure cannot turn it into an older command.
            selected, primary = first, first
            primary_status, secondary_status = -1, 0
            primary_iterations, secondary_iterations = 0, 0
            primary_ms = 1e3 * (perf_counter() - tracking_started)
            secondary_ms = 0.0
            violation = hard_error
        else:
            selected, primary = task.acceleration, task.primary_acceleration
            primary_status, secondary_status = task.primary_status, task.secondary_status
            primary_iterations, secondary_iterations = task.primary_iterations, task.secondary_iterations
            primary_ms, secondary_ms = task.primary_ms, task.secondary_ms
            violation = max(task.max_violation,
                            self._violation(selected, M, c, lower, upper, torque_limits))
        return HierarchicalQPResult(
            acceleration=selected.copy(), primary_acceleration=primary.copy(),
            primary_status=primary_status, secondary_status=secondary_status,
            primary_iterations=primary_iterations, secondary_iterations=secondary_iterations,
            max_violation=float(violation), primary_ms=primary_ms, secondary_ms=secondary_ms,
            soft_slack_lower=np.maximum(soft_lower - selected, 0.0),
            soft_slack_upper=np.maximum(selected - soft_upper, 0.0),
            safety_status=int(safety.info.status_val),
            safety_iterations=int(safety.info.iter), safety_ms=safety_ms)
