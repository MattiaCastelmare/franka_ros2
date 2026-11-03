"""utils.go_around: the go-around is smooth (rate limits), keeps its side, and
stops the task pressing into the obstacle."""
import numpy as np

from franka_experiments.utils.go_around import GoAroundState, go_around_step

KW = dict(engage_margin=0.15, speed=0.25, gain=3.0, max_accel=2.5, max_bias=8.0,
          horizon_s=0.3, damping=0.05, max_jerk=10.0)
DT = 0.01


def _jp(seed=0):
    return np.random.default_rng(seed).normal(size=(3, 7))


def test_zero_outside_engage_margin():
    st = GoAroundState()
    b, c = go_around_step(st, np.ones(7), np.zeros(7), _jp(), np.array([1., 0, 0]), 0.5, DT, **KW)
    assert np.allclose(b, 0) and np.allclose(c, 0) and st.t is None


def test_bias_rate_limited():
    st = GoAroundState(); prev = np.zeros(7)
    for _ in range(200):
        b, _ = go_around_step(st, np.zeros(7), np.zeros(7), _jp(), np.array([1., 0, 0]), 0.0, DT, **KW)
        assert np.linalg.norm(b - prev) <= KW['max_jerk'] * DT + 1e-9
        prev = b


def test_side_never_flips_on_opposite_intent():
    Jp = _jp(); n = np.array([1., 0, 0]); st = GoAroundState()
    pinv = np.linalg.pinv(Jp)
    go_around_step(st, pinv @ np.array([0, 1., 0]), np.zeros(7), Jp, n, 0.0, DT, **KW)
    side = st.t.copy()
    assert side[1] > 0.9
    for _ in range(100):   # task now asks for the other side
        go_around_step(st, pinv @ np.array([0, -1., 0]), np.zeros(7), Jp, n, 0.0, DT, **KW)
    assert float(st.t @ side) > 0.9


def test_side_released_only_past_release_margin():
    Jp = _jp(); n = np.array([1., 0, 0]); st = GoAroundState()
    go_around_step(st, np.zeros(7), np.zeros(7), Jp, n, 0.0, DT, **KW)
    go_around_step(st, np.zeros(7), np.zeros(7), Jp, n, 0.18, DT, **KW)   # 1.2·engage
    assert st.t is not None
    go_around_step(st, np.zeros(7), np.zeros(7), Jp, n, 0.30, DT, **KW)   # > 1.5·engage
    assert st.t is None


def test_inward_task_push_removed_tangential_kept():
    Jp = _jp(); n = np.array([1., 0, 0]); st = GoAroundState(w=1.0)
    qn = np.linalg.pinv(Jp) @ np.array([-1., 0.5, 0])
    _, c = go_around_step(st, qn, np.zeros(7), Jp, n, 0.0, DT, **KW)
    a = Jp @ (qn + c)
    assert abs(a[0]) < 0.02 and abs(a[1] - 0.5) < 0.02


def test_blind_fades_out():
    Jp = _jp(); n = np.array([1., 0, 0]); st = GoAroundState()
    for _ in range(100):
        go_around_step(st, np.zeros(7), np.zeros(7), Jp, n, 0.0, DT, **KW)
    for _ in range(300):
        b, c = go_around_step(st, np.zeros(7), np.zeros(7), None, None, float('inf'), DT, **KW)
    assert np.allclose(b, 0) and st.w == 0.0 and st.t is None
