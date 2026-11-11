"""Two-level acceleration QP with hard joint and actuator constraints.

The decision variable is the *actuator-induced* acceleration
``u = solve(M, tau - c)``. External disturbances belong in the bounds supplied
by the caller, not in an extra torque compensation term here. The primary
Cartesian action is solved first; the secondary solve preserves its attained
``J @ u`` exactly, up to a checked numerical equality tolerance.

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
