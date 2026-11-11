"""Find a fixed low-mass posture offline at the first circle/wall intersection.

Run with an explicit selection criterion and joint margin. The result is a
task YAML containing ``q_posture_ref`` and a separate JSON report. Nothing
is optimized in the control loop, and a static candidate does not establish
the mass or joint margins actually reached during a simulation.
"""
from __future__ import annotations

import argparse
import copy
import json
from pathlib import Path

import numpy as np
from scipy.optimize import brentq, minimize
import yaml

from fr3_sim.simulation.fr3_model import FR3Model, Q_MAX, Q_MIN

from fr3_sim.methods.circle_reference import CircleReference
from fr3_sim.methods.mass_objective import DirectionalMassObjective, projected_mass_torque


def _vector(value, length, name):
    value = np.asarray(value, dtype=float)
    if value.shape != (length,) or not np.all(np.isfinite(value)):
        raise ValueError(f'{name} must contain {length} finite values')
    return value


def _normal(wall):
    normal = _vector(wall['normal'], 3, 'wall normal')
    length = np.linalg.norm(normal)
    if length <= 0.0:
        raise ValueError('wall normal must be nonzero')
    return normal / length


def first_contact_reference(robot, task, wall):
    """Return position, time and inward speed at the first reference crossing.

    The outward signed gap is D + A*cos(theta) + B*sin(theta). Select its
    first descending root, then invert the circle's integrated startup ramp.
    This is reference geometry, not measured contact or observer detection.
    """
    initial = _vector(task['initial_q'], 7, 'initial_q')
    reference = CircleReference(robot.position(initial), **task.get('circle', {}))
    normal = _normal(wall)
    point = _vector(wall['point'], 3, 'wall point')
    radius = reference.radius
    d = float(normal @ (reference.center - point))
    a, b = -radius * normal[0], radius * normal[2]
    amplitude = float(np.hypot(a, b))
    if d + a <= 0.0:
        raise ValueError('the reference must start outside the wall')
    if amplitude == 0.0 or abs(d) >= amplitude:
        raise ValueError('the circle does not enter the wall plane')
    theta = float((np.arctan2(b, a) + np.arccos(-d / amplitude)) % (2 * np.pi))
    omega = 2 * np.pi / reference.period
    ramp = reference.ramp
    if ramp > 0.0 and theta < 0.5 * omega * ramp:
        def phase(t):
            u = t / ramp
            return omega * ramp * (u**3 - 0.5 * u**4)
        time = float(brentq(lambda t: phase(t) - theta, 0.0, ramp))
    else:
        time = float(theta / omega + 0.5 * ramp)
    position, velocity, _ = reference.sample(time)
    if abs(float(normal @ (position - point))) > 1e-9 or -normal @ velocity <= 0.0:
        raise ValueError('could not identify an entering reference crossing')
    return position, time, float(-normal @ velocity)


def _baseline_state(robot, task, wall, path):
    """Accept only a matching baseline-law run and its pre-contact sample."""
    with np.load(path, allow_pickle=False) as data:
        meta = json.loads(str(data['meta']))
        params = meta['params']
        controller = str(meta['controller']).replace('\\', '/').rsplit('/', 1)[-1]
        # Read historical zero-gain logs for posture provenance only; the
        # optimized controller itself is no longer part of the active package.
        if controller not in ('baseline.py', 'controller_baseline.py', 'controller_optimized.py'):
            raise ValueError('baseline log must use baseline or optimized with zero mass gain')
        if controller == 'controller_optimized.py' and float(
                params.get('mass_minimization', {}).get('gain', 0.01)) != 0.0:
            raise ValueError('the selected optimized log has a nonzero mass gain')
        initial = _vector(task['initial_q'], 7, 'initial_q')
        if not np.allclose(params['initial_q'], initial, rtol=0.0, atol=1e-12):
            raise ValueError('baseline initial_q differs from the task')
        if not np.allclose(params.get('q_posture_ref', params['initial_q']), initial,
                           rtol=0.0, atol=1e-12):
            raise ValueError('baseline log must use initial_q as its posture reference')
        defaults = {
            'circle': {'radius': 0.04, 'period': 8.0, 'ramp': 1.0},
            'tracking': {'kp_position': 100.0, 'kd_position': 20.0,
                         'kp_nullspace': 25.0, 'kd_nullspace': 10.0},
        }
        for section, entries in defaults.items():
            for key, default in entries.items():
                logged = float(params.get(section, {}).get(key, default))
                requested = float(task.get(section, {}).get(key, default))
                if not np.isclose(logged, requested, rtol=0.0, atol=1e-12):
                    raise ValueError(f'baseline {section}.{key} differs from the task')
        logged_wall = meta['wall']
        if (not logged_wall.get('enabled', True)
                or not np.allclose(logged_wall['point'], wall['point'], rtol=0.0, atol=1e-12)
                or not np.allclose(_normal(logged_wall), _normal(wall), rtol=0.0, atol=1e-12)):
            raise ValueError('baseline wall geometry differs from the selected wall')
        t = np.asarray(data['t'], dtype=float)
        force = np.asarray(data['F_n'], dtype=float)
        mass = np.asarray(data['user_mass_kg'], dtype=float).reshape(-1)
        q = np.asarray(data['q'], dtype=float)
        if (t.ndim != 1 or force.shape != t.shape or mass.shape != t.shape
                or q.shape != (len(t), 7) or not np.all(np.isfinite(t))
                or np.any(np.diff(t) <= 0.0)):
            raise ValueError('invalid baseline log dimensions or timestamps')
        contact = np.flatnonzero(force > 0.0)
        if not contact.size or contact[0] == 0:
            raise ValueError('baseline log needs a sample before its first contact')
        index = int(contact[0]) - 1
        seed = _vector(q[index], 7, 'baseline pre-contact configuration').copy()
        baseline_mass = float(mass[index])
        recomputed = float(robot.effective_mass(seed, _normal(wall)))
        if not np.isfinite(baseline_mass) or baseline_mass <= 0.0 or not np.isclose(
                baseline_mass, recomputed, rtol=1e-7, atol=1e-8):
            raise ValueError('logged baseline mass does not match the current model at its sample')
        return seed, baseline_mass, float(t[index]), float(t[index + 1])


def optimize_posture(robot, task, wall, baseline_log, *, selection, joint_margin,
                     reduction=0.30):
    """Find one local candidate, returning its joint vector and static report.

    ``closest-target`` minimizes squared joint displacement from the logged
    baseline contact configuration, subject to the requested mass reduction.
    ``minimum-mass`` minimizes mass directly. Both retain the Cartesian contact
    point and explicit position margins; neither is a global-optimum claim.
    """
    if selection not in ('closest-target', 'minimum-mass'):
        raise ValueError('selection must be closest-target or minimum-mass')
    joint_margin, reduction = float(joint_margin), float(reduction)
    # Runner stops within 0.05 rad of a physical joint-position bound.
    if not np.isfinite(joint_margin) or joint_margin <= 0.05:
        raise ValueError('joint_margin must be greater than the runner stop margin, 0.05 rad')
    if not np.isfinite(reduction) or not 0.0 < reduction < 1.0:
        raise ValueError('reduction must lie strictly between zero and one')
    lower, upper = Q_MIN + joint_margin, Q_MAX - joint_margin
    if np.any(lower >= upper):
        raise ValueError('joint margin leaves no admissible joint range')
    position, time, speed = first_contact_reference(robot, task, wall)
    normal = _normal(wall)
    seed, baseline_mass, seed_time, logged_contact_time = _baseline_state(
        robot, task, wall, baseline_log)
    target = (1.0 - reduction) * baseline_mass
    objective = DirectionalMassObjective(robot, normal)
    cached_q, cached_result = None, None

    def mass_and_gradient(q):
        nonlocal cached_q, cached_result
        if cached_q is None or not np.array_equal(q, cached_q):
            cached_result = objective.evaluate(q)
            cached_q = np.asarray(q, dtype=float).copy()
        return cached_result

    constraints = [{'type': 'eq', 'fun': lambda q: robot.position(q) - position,
                    'jac': robot.jacobian_pos}]
    if selection == 'closest-target':
        fun = lambda q: 0.5 * float((q - seed) @ (q - seed))
        jac = lambda q: q - seed
        constraints.append({'type': 'ineq', 'fun': lambda q: target - mass_and_gradient(q)[0],
                            'jac': lambda q: -mass_and_gradient(q)[1]})
    else:
        fun = lambda q: mass_and_gradient(q)[0]
        jac = lambda q: mass_and_gradient(q)[1]
    result = minimize(fun, np.clip(seed, lower, upper), jac=jac, method='SLSQP',
                      bounds=list(zip(lower, upper)), constraints=constraints,
                      options={'ftol': 1e-10, 'maxiter': 1000})
    candidate = _vector(result.x, 7, 'optimized posture')
    mass, gradient = objective.evaluate(candidate)
    error = float(np.linalg.norm(robot.position(candidate) - position))
    valid = (bool(result.success) and np.isfinite(mass) and mass <= target + 1e-7
             and error <= 1e-7 and np.all(candidate >= lower - 1e-8)
             and np.all(candidate <= upper + 1e-8))
    if not valid:
        raise RuntimeError(f'no acceptable local posture: {result.message}; '
                           f'mass={mass:.9g} kg, target={target:.9g} kg, position error={error:.3g} m')
    margins = np.minimum(candidate - Q_MIN, Q_MAX - candidate)
    report = {
        'nominal_candidate_only': True, 'achieved_in_simulation': False,
        'selection': selection, 'joint_margin_rad': joint_margin,
        'requested_reduction': reduction, 'baseline_log': str(Path(baseline_log).resolve()),
        'baseline_precontact_time_s': seed_time, 'baseline_first_contact_time_s': logged_contact_time,
        'baseline_mass_kg': baseline_mass, 'baseline_recomputed_mass_kg':
            float(robot.effective_mass(seed, normal)),
        'target_mass_kg': target, 'candidate_mass_kg': float(mass),
        'candidate_reduction': float(1.0 - mass / baseline_mass),
        'initial_q': np.asarray(task['initial_q'], dtype=float).tolist(),
        'baseline_precontact_q': seed.tolist(), 'q_posture_ref': candidate.tolist(),
        'contact_position_m': position.tolist(), 'normal': normal.tolist(),
        'reference_contact_time_s': time, 'reference_entry_speed_mps': speed,
        'position_error_m': error, 'joint_margins_rad': margins.tolist(),
        'minimum_joint_margin_rad': float(np.min(margins)),
        'solver': {'method': 'SLSQP', 'success': bool(result.success),
                   'message': str(result.message), 'iterations': int(result.nit)},
    }
    kp = float(task.get('tracking', {}).get('kp_nullspace', 25.0))
    gain = float(task.get('mass_minimization', {}).get('gain', 0.01))
    if not np.isfinite(gain) or gain < 0.0:
        raise ValueError('mass_minimization.gain must be finite and nonnegative')
    if kp > 0.0:
        M, J = robot.mass_matrix(candidate), robot.jacobian_pos(candidate)
        try:
            offset = np.linalg.solve(M, projected_mass_torque(M, J, gradient, gain)) / kp
            equilibrium = candidate + offset
            report['local_constant_geometry_estimate'] = {
                'description': 'Online mass/posture balance only; not a trajectory or limit guarantee.',
                'joint_offset_rad': offset.tolist(),
                'minimum_joint_margin_rad': float(np.min(np.minimum(
                    equilibrium - Q_MIN, Q_MAX - equilibrium))),
            }
        except np.linalg.LinAlgError:
            report['local_constant_geometry_estimate'] = None
    return candidate.copy(), report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--task-config', required=True, type=Path)
    parser.add_argument('--wall-config', required=True, type=Path)
    parser.add_argument('--baseline-log', required=True, type=Path)
    parser.add_argument('--selection', required=True, choices=('closest-target', 'minimum-mass'))
    parser.add_argument('--joint-margin', required=True, type=float, help='physical joint-bound margin [rad]')
    parser.add_argument('--reduction', type=float, default=0.30, help='requested fractional mass reduction')
    parser.add_argument('--output', required=True, type=Path, help='new task YAML; no input file is edited implicitly')
    parser.add_argument('--report', type=Path, help='JSON report; default: OUTPUT.posture.json')
    args = parser.parse_args()
    report_path = args.report or args.output.with_suffix('.posture.json')
    if args.output.resolve() == report_path.resolve():
        parser.error('task output and report must be different files')
    try:
        task = yaml.safe_load(args.task_config.read_text())
        wall = yaml.safe_load(args.wall_config.read_text())
        candidate, report = optimize_posture(
            FR3Model(), task, wall, args.baseline_log, selection=args.selection,
            joint_margin=args.joint_margin, reduction=args.reduction)
        output = copy.deepcopy(task)
        output['q_posture_ref'] = candidate.tolist()
        task_text = yaml.safe_dump(output, sort_keys=False)
        report_text = json.dumps(report, indent=2, allow_nan=False) + '\n'
        args.output.parent.mkdir(parents=True, exist_ok=True)
        report_path.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(task_text)
        report_path.write_text(report_text)
    except (ValueError, RuntimeError, OSError, KeyError, np.linalg.LinAlgError) as exc:
        parser.exit(2, f'{exc}\n')
    print(f'task: {args.output}\nreport: {report_path}')
    print(f'static candidate mass: {report["candidate_mass_kg"]:.6f} kg '
          f'({100 * report["candidate_reduction"]:.2f}% below baseline)')
    print(f'minimum physical joint margin: {report["minimum_joint_margin_rad"]:.6f} rad')


if __name__ == '__main__':
    main()
