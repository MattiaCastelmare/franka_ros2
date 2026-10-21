#!/usr/bin/env python3
"""Visual check of utils/mpc_problem.py: closed-loop runs on the double integrator, plotted.

No ROS and no robot: the MPC drives the 7 joints of the exact model it plans with,
so the plots show the QP itself, before kinematics and perception come in. One PNG
per scenario, each with position, velocity, acceleration and solver effort of joint 1:

goal      step to a joint goal. The receding-horizon plans of some cycles are drawn
          faded on the position: each starts where the joint is and ends almost at
          rest (soft terminal stop).
barrier   a "human row" on joint 1: q1 must stay above a barrier b(t) that approaches
          at constant speed, stops, then withdraws. 'predicted' extrapolates b along
          the horizon at its current speed (the constant-velocity forecast of the
          tracker); 'static' freezes it at its current value (mpc_static, ablation
          A1). Predicting moves the joint earlier and with less acceleration.
limits    starting faster than the velocity limit, toward a position limit: the
          relaxed rows let the MPC brake as hard as it can instead of failing.

The terminal summary gives the solver effort per scenario: with --solver osqp
the barrier needs thousands of ADMM iterations, with --solver daqp (pip install
daqp) the same QPs take about a millisecond.

USAGE (inside the ROS container, workspace sourced, this checkout on PYTHONPATH)
    python3 scripts/mpc_problem_demo.py --out /ros2_ws/src/mpc_demo [--scenario barrier]
                                        [--solver daqp] [--show]
"""

from __future__ import annotations

import argparse
import os

import numpy as np

from franka_experiments.utils.mpc_problem import MPCProblem
from franka_experiments.utils.mpc_problem import MPCSettings

N_JOINTS = 7
W_JOINT = 1000.0            # weight of the joint goal, as w_position for the TCP
Q_LIM, QD_MAX, QDD_MAX = 2.5, 2.0, 5.0
SOLVER = 'osqp'             # set from --solver


def make_problem(**settings) -> MPCProblem:
    ones = np.ones(N_JOINTS)
    return MPCProblem(MPCSettings(**{'solver': SOLVER, **settings}), -Q_LIM * ones, Q_LIM * ones,
                      QD_MAX * ones, QDD_MAX * ones)


def run(problem: MPCProblem, duration: float, q0, qd0, goal, rows=None) -> dict:
    """Closed loop: solve, apply u_0 for one dt on the exact model, repeat.

    rows(problem, t, q_bar) may set human rows; a failed solve brakes, as the node would.
    """
    q, qd, u_prev = np.array(q0, float), np.array(qd0, float), np.zeros(N_JOINTS)
    U = np.zeros(problem.nu)
    log = {key: [] for key in ('t', 'q', 'qd', 'u', 'plan', 'ms', 'it', 'clr', 'slack', 'failed')}
    for cycle in range(int(round(duration / problem.dt))):
        t = cycle * problem.dt
        U_bar = problem.shift_plan(U)
        problem.begin_cycle(q, qd, u_prev)
        for k in range(1, problem.n_steps + 1):
            problem.add_tracking_term(k, np.eye(N_JOINTS), goal, W_JOINT * np.eye(N_JOINTS))
        if rows is not None:
            q_bar, _ = problem.rollout(q, qd, U_bar)
            rows(problem, t, q_bar)
        solution = problem.solve(U_bar)
        failed = solution is None
        if failed:
            U = np.tile(-np.clip(qd / problem.dt, -QDD_MAX, QDD_MAX), problem.n_steps)
        else:
            U = solution
        u_prev = problem.first_input(U)
        plan, _ = problem.rollout(q, qd, U)

        d = problem.diag
        for key, value in (('t', t), ('q', q.copy()), ('qd', qd.copy()), ('u', u_prev.copy()),
                           ('plan', plan), ('ms', d.solve_time_ms), ('it', d.iterations),
                           ('clr', d.human_clearance), ('slack', d.max_slack), ('failed', failed)):
            log[key].append(value)

        x = problem.A_dyn @ np.concatenate([q, qd]) + problem.B_dyn @ u_prev
        q, qd = x[:N_JOINTS], x[N_JOINTS:]
    return {key: np.array(value) for key, value in log.items()}


# ── Scenarios ───────────────────────────────────────────────────────────────

def scenario_goal():
    p = make_problem()
    goal = np.linspace(-0.4, 1.0, N_JOINTS)[::-1]   # joint 1 travels 1 rad
    return {'run': run(p, 3.0, np.zeros(N_JOINTS), np.zeros(N_JOINTS), goal), 'problem': p, 'goal': goal}


def barrier(t: float):
    """Barrier on joint 1 [rad] and its speed: approach at 0.4 rad/s, hold, withdraw."""
    if t < 2.25:
        return -0.6 + 0.4 * t, 0.4
    if t < 3.5:
        return 0.3, 0.0
    return 0.3 - 0.4 * (t - 3.5), -0.4


def barrier_rows(predicted: bool, margin: float = 0.0):
    """One human row per step: q1(t_k) ≥ b(t_k), with b predicted or frozen at t."""
    def rows(problem: MPCProblem, t: float, q_bar: np.ndarray) -> None:
        b, b_dot = barrier(t)
        a = np.zeros((1, N_JOINTS))
        a[0, 0] = 1.0   # the "distance" grows with q1: d = q1 − b + d_safe + δ
        for k in range(1, problem.n_steps + 1):
            b_k = b + b_dot * k * problem.dt if predicted else b
            d_bar = q_bar[k - 1, 0] - b_k + problem.cfg.d_safe + problem.cfg.delta
            problem.set_human_rows(k, a, [d_bar], [margin], q_bar[k - 1])
    return rows


def scenario_barrier():
    goal = np.zeros(N_JOINTS)
    q0 = np.zeros(N_JOINTS)
    runs = {name: run(make_problem(), 5.0, q0, q0, goal, barrier_rows(predicted))
            for name, predicted in (('predicted', True), ('static', False))}
    return {'runs': runs, 'goal': goal}


def scenario_limits():
    p = make_problem()
    q0 = np.full(N_JOINTS, 2.2)
    qd0 = np.full(N_JOINTS, 1.6)   # above qdot_scale · q̇_max = 1.4, toward q_max − margin = 2.45
    goal = np.full(N_JOINTS, 2.3)
    return {'run': run(p, 2.0, q0, qd0, goal), 'problem': p, 'goal': goal}


# ── Plots ───────────────────────────────────────────────────────────────────

def _effort(ax, runs: dict) -> None:
    for name, r in runs.items():
        ax.plot(r['t'], r['ms'], label=f'{name}: {SOLVER} solve [ms]')
    twin = ax.twinx()
    for name, r in runs.items():
        twin.plot(r['t'], r['it'], ':', label=f'{name}: iterations')
    ax.set_ylabel(f'{SOLVER} solve [ms]')
    twin.set_ylabel('iterations')
    ax.set_xlabel('t [s]')
    ax.legend(loc='upper left', fontsize=8)
    twin.legend(loc='upper right', fontsize=8)


def plot_single(plt, title: str, data: dict, path: str, plan_every: int = 6) -> None:
    r, p = data['run'], data['problem']
    c = p.cfg
    fig, axes = plt.subplots(2, 2, figsize=(13, 8), sharex=True)
    ax = axes[0, 0]
    ax.plot(r['t'], r['q'][:, 0], 'k', lw=2, label='q1 executed')
    for i in range(0, len(r['t']), plan_every):
        t_plan = r['t'][i] + p.dt * np.arange(1, p.n_steps + 1)
        ax.plot(t_plan, r['plan'][i][:, 0], color='tab:purple', alpha=0.5,
                label='plans (every %d cycles)' % plan_every if i == 0 else None)
    ax.axhline(data['goal'][0], color='tab:green', ls='--', label='goal')
    ax.axhline(Q_LIM - c.joint_margin, color='tab:red', ls=':', label='q_max − margin')
    ax.set_ylabel('q1 [rad]')
    ax.legend(fontsize=8)
    ax = axes[0, 1]
    ax.plot(r['t'], r['qd'][:, 0], 'k')
    for sign in (1, -1):
        ax.axhline(sign * c.qdot_scale * QD_MAX, color='tab:red', ls=':')
    ax.set_ylabel('q̇1 [rad/s]  (dotted: qdot_scale · q̇_max)')
    ax = axes[1, 0]
    ax.step(r['t'], r['u'][:, 0], 'k', where='post')
    for sign in (1, -1):
        ax.axhline(sign * QDD_MAX, color='tab:red', ls=':')
    ax.set_ylabel('q̈1 applied [rad/s²]')
    ax.set_xlabel('t [s]')
    _effort(axes[1, 1], {'mpc': r})
    fig.suptitle(title)
    fig.tight_layout()
    fig.savefig(path, dpi=110)


def plot_barrier(plt, data: dict, path: str) -> None:
    runs = data['runs']
    t = runs['predicted']['t']
    b = np.array([barrier(ti)[0] for ti in t])
    colors = {'predicted': 'tab:blue', 'static': 'tab:orange'}
    fig, axes = plt.subplots(2, 2, figsize=(13, 8), sharex=True)
    ax = axes[0, 0]
    ax.fill_between(t, -1.0, b, color='tab:red', alpha=0.15, label='barrier: q1 must stay above')
    for name, r in runs.items():
        ax.plot(r['t'], r['q'][:, 0], color=colors[name], lw=2, label=f'q1 {name}')
    ax.set_ylim(-1.0, 0.8)
    ax.set_ylabel('q1 [rad]')
    ax.legend(fontsize=8)
    ax = axes[0, 1]
    for name, r in runs.items():
        ax.plot(r['t'], r['clr'], color=colors[name], label=f'{name}: min clearance')
        ax.plot(r['t'], r['slack'], ':', color=colors[name], label=f'{name}: max slack')
    ax.axhline(0.0, color='k', lw=0.8)
    ax.set_ylabel('[rad]  over the horizon')
    ax.legend(fontsize=8)
    ax = axes[1, 0]
    for name, r in runs.items():
        ax.step(r['t'], r['u'][:, 0], color=colors[name], where='post', label=name)
    ax.set_ylabel('q̈1 applied [rad/s²]')
    ax.set_xlabel('t [s]')
    ax.legend(fontsize=8)
    _effort(axes[1, 1], runs)
    # While the barrier approaches, before its stop surprises the constant-velocity forecast
    approach = t < 2.2
    peak = {name: np.abs(r['u'][approach, 0]).max() for name, r in runs.items()}
    fig.suptitle(f'barrier on joint 1 — {SOLVER} — peak |q̈1| while it approaches: ' +
                 ', '.join(f'{name} {value:.2f} rad/s²' for name, value in peak.items()))
    fig.tight_layout()
    fig.savefig(path, dpi=110)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    parser.add_argument('--out', default='mpc_demo', help='output folder for the PNGs')
    parser.add_argument('--scenario', choices=('goal', 'barrier', 'limits', 'all'), default='all')
    parser.add_argument('--solver', choices=('osqp', 'daqp'), default='osqp')
    parser.add_argument('--show', action='store_true', help='also open the figures')
    args = parser.parse_args()
    global SOLVER
    SOLVER = args.solver

    import matplotlib
    if not args.show:
        matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    os.makedirs(args.out, exist_ok=True)
    chosen = ('goal', 'barrier', 'limits') if args.scenario == 'all' else (args.scenario,)
    for name in chosen:
        path = os.path.join(args.out, f'mpc_{name}_{SOLVER}.png')
        if name == 'goal':
            data = scenario_goal()
            plot_single(plt, f'step to a joint goal (joint 1) — {SOLVER}', data, path)
            runs = {'mpc': data['run']}
        elif name == 'limits':
            data = scenario_limits()
            plot_single(plt, f'too fast toward a position limit (joint 1) — {SOLVER}', data, path,
                        plan_every=4)
            runs = {'mpc': data['run']}
        else:
            data = scenario_barrier()
            plot_barrier(plt, data, path)
            runs = data['runs']
        for run_name, r in runs.items():
            print(f'{name:8s} {run_name:9s} solve ms p50/p99/max {np.median(r["ms"]):.2f}/'
                  f'{np.percentile(r["ms"], 99):.2f}/{r["ms"].max():.2f}  iterations p50/max '
                  f'{np.median(r["it"]):.0f}/{r["it"].max()}  failed {int(r["failed"].sum())}')
        print(f'  -> {path}')
    if args.show:
        plt.show()


if __name__ == '__main__':
    main()