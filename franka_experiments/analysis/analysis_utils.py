"""Shared helpers of the a-posteriori analyses of the experiment logs (human_logging CSVs).

Used by kf_compare.py (Kalman velocity against a Savitzky-Golay reference),
whiteness_test.py (innovation whiteness and NIS consistency) and mpc_analysis.py (MPC
prediction residuals, executed acceleration, plan consistency, cost and multipliers).
What real logs throw at an analysis is handled here, once:
- a bag replayed in loop: the stamps jump back and the log holds several passes over the
  same data (only one pass is analysed, the same one in every file of the run);
- duplicated or slightly reordered stamps, dropped frames (uneven sampling, gaps);
- keypoints logged as 0 while not valid, innovations logged as 0 on frames with no update;
- empty or truncated files, single-arm runs (experiment_<suffix>.csv) and dual-arm ones
  (experiment_<side>_<suffix>.csv), logs written before the nis / gated columns existed.

The default folders work on the host (~/Desktop/Thesis) and in the container, which mounts
them on /ros2_ws/experiment_logs and /ros2_ws/graphs_thesis (docker-compose.yml).
"""

from __future__ import annotations

import re
import sys
from pathlib import Path

import numpy as np
import pandas as pd
from scipy.stats import chi2

LOG_DIRS = ("~/Desktop/Thesis/experiment_logs", "/ros2_ws/experiment_logs")   # host, container
LOG_DIR = next((d for d in LOG_DIRS if Path(d).expanduser().is_dir()), LOG_DIRS[0])
GRAPH_DIR = str(Path(LOG_DIR).parent / "graphs_thesis")   # next to the logs
KEYPOINTS = ("shoulder", "elbow", "wrist", "hand")
AXES = ("x", "y", "z")
RUN_NAME = re.compile(r"^\d{8}_\d{6}$")
LOOP_JUMP_S = 1.0     # a stamp this far behind the previous one starts a new pass (bag loop)
GAP_FACTOR = 1.5      # spacing above GAP_FACTOR x the frame period: a gap, never bridged
MATCH_FACTOR = 0.25   # two stamps closer than MATCH_FACTOR x the frame period: the same frame


def fail(message: str) -> None:
    print(f"Error: {message}", file=sys.stderr)
    sys.exit(1)


def warn(message: str) -> None:
    print(f"Warning: {message}")


# ── Runs and files ─────────────────────────────────────────────────────────────

def resolve_run(log_dir: str, run_name: str | None, controllers: tuple[str, ...] | None = None) -> Path:
    """The run folder: the one named, else the latest YYYYMMDD_HHMMSS one.

    Runs are in log_dir/<controller>/<run> (human, nominal, cbf, mpc, ...) or, the
    older ones, directly in log_dir/<run>. run_name may also be '<controller>/<run>'.
    controllers: where to look for the latest run (e.g. ('mpc', 'mpc_cbf')); None = anywhere.
    """
    base = Path(log_dir).expanduser()
    if not base.is_dir():
        fail(f"log directory '{base}' does not exist.")
    candidates = [d for d in base.iterdir() if d.is_dir()]
    candidates += [r for d in candidates if not RUN_NAME.match(d.name) for r in d.iterdir() if r.is_dir()]
    if run_name:
        matches = [d for d in candidates if d.name == run_name or d.relative_to(base).as_posix() == run_name]
        if not matches:
            fail(f"run '{run_name}' not found in '{base}' or its controller subfolders.")
        if len(matches) > 1:
            fail(f"run '{run_name}' is in several folders: "
                 f"{[m.relative_to(base).as_posix() for m in matches]}; name it as '<controller>/<run>'.")
        return matches[0]
    runs = sorted((d for d in candidates if RUN_NAME.match(d.name)
                   and (controllers is None or d.parent.name in controllers)), key=lambda d: d.name)
    if not runs:
        where = "its controller subfolders" if controllers is None else f"{'/'.join(controllers)}"
        fail(f"no run folder (YYYYMMDD_HHMMSS) in '{base}' or {where}.")
    print(f"No run name given: using the latest run, {runs[-1].relative_to(base).as_posix()}")
    return runs[-1]


def find_arms(run: Path, side: str, suffixes: tuple[str, ...]) -> list[tuple[str, dict]]:
    """[(label, {suffix: path})] for each arm of the run that has every file in suffixes.

    Dual-arm runs name their files experiment_<side>_<suffix>; a single-arm run's
    experiment_<suffix> files are analysed once, labelled 'arm'.
    """
    found = []
    for s in (("left", "right") if side == "both" else (side,)):
        paths = {suffix: run / f"experiment_{s}_{suffix}" for suffix in suffixes}
        if all(p.is_file() for p in paths.values()):
            found.append((s, paths))
    if not found:
        paths = {suffix: run / f"experiment_{suffix}" for suffix in suffixes}
        if all(p.is_file() for p in paths.values()):
            found.append(("arm", paths))
    return found


def read_numeric_csv(path: Path) -> pd.DataFrame | None:
    """The CSV as numbers (anything else -> NaN), rows without a finite stamp dropped."""
    try:
        df = pd.read_csv(path, on_bad_lines="skip")
    except (pd.errors.EmptyDataError, pd.errors.ParserError, UnicodeDecodeError, OSError) as exc:
        warn(f"cannot read {path.name}: {exc}")
        return None
    if df.empty or "timestamp" not in df.columns:
        warn(f"{path.name} holds no data.")
        return None
    df = df.apply(pd.to_numeric, errors="coerce").replace([np.inf, -np.inf], np.nan)
    df = df[df["timestamp"].notna()]
    if df.empty:
        warn(f"{path.name} holds no valid stamp.")
        return None
    return df


def split_passes(df: pd.DataFrame) -> list[pd.DataFrame]:
    """The log cut where the stamps jump back by more than LOOP_JUMP_S (a bag loop)."""
    t = df["timestamp"].to_numpy()
    cuts = np.flatnonzero(np.diff(t) < -LOOP_JUMP_S) + 1
    return [df.iloc[a:b] for a, b in zip(np.r_[0, cuts], np.r_[cuts, len(t)])]


def load_arm(paths: dict) -> dict | None:
    """The logs of one arm, each sorted, without duplicated stamps, and reduced to one pass.

    With a looped bag every file holds the same passes: the longest pass of the first file
    is taken in all of them (each file's own longest if their pass counts differ).
    """
    passes = {}
    for key, path in paths.items():
        df = read_numeric_csv(path)
        if df is None:
            return None
        passes[key] = split_passes(df)
    first = next(iter(passes.values()))
    pick = int(np.argmax([len(p) for p in first]))
    same_count = len({len(p) for p in passes.values()}) == 1
    if len(first) > 1:
        warn(f"the stamps restart {len(first) - 1} time(s) (bag played in loop?): analysing "
             f"pass {pick + 1} of {len(first)}, the longest.")
    logs = {}
    for key, p in passes.items():
        chosen = p[pick] if same_count else max(p, key=len)
        chosen = chosen.sort_values("timestamp", kind="stable")
        logs[key] = chosen.drop_duplicates("timestamp", keep="last").reset_index(drop=True)
    return logs


# ── Arrays out of the three logs ───────────────────────────────────────────────

def columns(df: pd.DataFrame, names: list[str]) -> np.ndarray:
    """(N, len(names)) floats; a missing column is all NaN."""
    return np.stack([df[n].to_numpy(dtype=float) if n in df.columns
                     else np.full(len(df), np.nan) for n in names], axis=1)


def keypoints_xyz(df: pd.DataFrame, pattern: str) -> np.ndarray:
    """(N, 4, 3): pattern.format(kp=..., axis=...) for every keypoint and axis."""
    return np.stack([columns(df, [pattern.format(kp=kp, axis=a) for a in AXES])
                     for kp in KEYPOINTS], axis=1)


def raw_arrays(df: pd.DataFrame) -> dict:
    """Raw (pre-filter) positions; a missing keypoint is NaN (or the (0, 0, 0) placeholder)."""
    pos = keypoints_xyz(df, "{kp}_{axis}")
    pos[np.all(pos == 0.0, axis=-1)] = np.nan
    return dict(t=df["timestamp"].to_numpy(dtype=float), pos=pos)


def state_arrays(df: pd.DataFrame) -> dict:
    """Filtered positions and velocities, NaN where the keypoint is not valid."""
    pos = keypoints_xyz(df, "{kp}_{axis}")
    vel = keypoints_xyz(df, "{kp}_v{axis}")
    if "valid_0" in df.columns:
        valid = columns(df, [f"valid_{i}" for i in range(4)]) == 1
    else:   # oldest logs: an invalid keypoint is the (0, 0, 0) placeholder
        valid = ~np.all(pos == 0.0, axis=-1)
    measured = columns(df, [f"measured_{i}" for i in range(4)]) == 1
    pos[~valid] = np.nan
    vel[~valid] = np.nan
    return dict(t=df["timestamp"].to_numpy(dtype=float), pos=pos, vel=vel, valid=valid,
                measured=measured & valid)


def diag_arrays(df: pd.DataFrame) -> dict:
    """Innovations, trace(P) (NaN while the filter is not running) and, if logged, NIS / gated."""
    trace = columns(df, [f"{kp}_p_trace" for kp in KEYPOINTS])
    trace[~(trace > 0.0)] = np.nan
    has_nis = f"{KEYPOINTS[0]}_nis" in df.columns
    has_gated = f"{KEYPOINTS[0]}_gated" in df.columns
    return dict(
        t=df["timestamp"].to_numpy(dtype=float),
        inn=keypoints_xyz(df, "{kp}_inn_{axis}"),
        trace=trace,
        nis=columns(df, [f"{kp}_nis" for kp in KEYPOINTS]) if has_nis else None,
        gated=(columns(df, [f"{kp}_gated" for kp in KEYPOINTS]) == 1) if has_gated else None,
    )


# ── Time ──────────────────────────────────────────────────────────────────────

def frame_period(t: np.ndarray) -> float:
    """Median spacing of the stamps: the nominal frame period, blind to drops and gaps."""
    d = np.diff(np.asarray(t, dtype=float))
    d = d[d > 0.0]
    return float(np.median(d)) if d.size else float("nan")


def match(t_from: np.ndarray, t_to: np.ndarray, tol: float) -> np.ndarray:
    """For each stamp of t_to, the index of the nearest stamp of t_from within tol, else -1.

    t_from must be sorted.
    """
    t_from, t_to = np.asarray(t_from, dtype=float), np.asarray(t_to, dtype=float)
    n = len(t_from)
    if n == 0:
        return np.full(t_to.shape, -1)
    j = np.searchsorted(t_from, t_to)                  # t_from[j-1] < t_to <= t_from[j]
    lo, hi = np.clip(j - 1, 0, n - 1), np.clip(j, 0, n - 1)
    nearest = np.where(np.abs(t_from[lo] - t_to) <= np.abs(t_from[hi] - t_to), lo, hi)
    return np.where(np.abs(t_from[nearest] - t_to) <= tol, nearest, -1)


def interp_within(t_src: np.ndarray, y_src: np.ndarray, t_query: np.ndarray,
                  max_gap: float) -> np.ndarray:
    """y_src (N, k) linearly interpolated at t_query, never across a gap wider than max_gap.

    Rows of y_src with a NaN are dropped first; outside the samples, or across a gap: NaN.
    """
    y_src = np.asarray(y_src, dtype=float).reshape(len(t_src), -1)
    ok = np.isfinite(y_src).all(axis=1)
    ts, ys = np.asarray(t_src, dtype=float)[ok], y_src[ok]
    t_query = np.asarray(t_query, dtype=float)
    out = np.full((len(t_query), y_src.shape[1]), np.nan)
    n = len(ts)
    if n == 0:
        return out
    j = np.searchsorted(ts, t_query, side="right")     # ts[j-1] <= t_query < ts[j]
    lo, hi = np.clip(j - 1, 0, n - 1), np.clip(j, 0, n - 1)
    exact = (j > 0) & (ts[lo] == t_query)
    inside = (j > 0) & (j < n) & (ts[hi] - ts[lo] <= max_gap)
    span = np.where(inside, ts[hi] - ts[lo], 1.0)
    w = ((t_query - ts[lo]) / span)[:, None]
    blend = (1.0 - w) * ys[lo] + w * ys[hi]
    out[inside] = blend[inside]
    out[exact] = ys[lo][exact]
    return out


# ── Statistics ────────────────────────────────────────────────────────────────

def local_poly_derivative(t: np.ndarray, y: np.ndarray, window_s: float,
                          order: int = 2, t_eval: np.ndarray | None = None) -> np.ndarray:
    """dy/dt from a centred least-squares polynomial of `order` over [t - w/2, t + w/2].

    This is the Savitzky-Golay derivative written on the actual stamps: on evenly spaced
    samples it equals scipy's savgol_filter(y, window, order, deriv=1, delta=dt) with the
    same window; with dropped frames or gaps it still fits the samples where they are.
    A sample gets NaN unless its window holds at least order + 2 samples and reaches at
    least w/4 on both sides (no one-sided fit, i.e. no extrapolated slope at a gap).
    y: (N,) or (N, k); rows with a NaN are not used.
    t_eval: centre the windows on these times instead of on every sample (a 1 kHz log
    evaluated on a coarser grid); the result then has one row per t_eval.
    """
    t = np.asarray(t, dtype=float)
    y = np.asarray(y, dtype=float)
    flat = y.ndim == 1
    y = y.reshape(len(t), -1)
    ok = np.isfinite(y).all(axis=1) & np.isfinite(t)
    ts, ys = t[ok], y[ok]
    centres = ts if t_eval is None else np.asarray(t_eval, dtype=float)
    res = np.full((len(centres), y.shape[1]), np.nan)
    half = 0.5 * window_s
    lo = np.searchsorted(ts, centres - half - 1e-9, side="left")
    hi = np.searchsorted(ts, centres + half + 1e-9, side="right")
    for j, centre in enumerate(centres):
        a, b = lo[j], hi[j]
        if b - a < order + 2:
            continue
        x = ts[a:b] - centre
        if x[0] > -0.5 * half or x[-1] < 0.5 * half:
            continue
        X = np.vander(x / half, order + 1, increasing=True)
        coef, *_ = np.linalg.lstsq(X, ys[a:b], rcond=None)
        res[j] = coef[1] / half
    if t_eval is None:
        out = np.full(y.shape, np.nan)
        out[np.flatnonzero(ok)] = res
    else:
        out = res
    return out[:, 0] if flat else out


def acf_gapped(t: np.ndarray, y: np.ndarray, max_lag: int, period: float):
    """Sample autocorrelation of y at lags 1..max_lag frames, pairing samples by their stamps.

    t, y: the samples (sorted, finite). A pair at lag l is two samples l frame periods apart
    (within MATCH_FACTOR of a period), so dropped frames never shift a lag. Returns rho (L,)
    and the number of pairs per lag (NaN rho where fewer than 10 pairs).
    """
    t = np.asarray(t, dtype=float)
    yc = np.asarray(y, dtype=float) - np.mean(y)
    var = np.mean(yc**2)
    rho, pairs = np.full(max_lag, np.nan), np.zeros(max_lag, dtype=int)
    if len(yc) < 3 or var <= 0.0:
        return rho, pairs
    for lag in range(1, max_lag + 1):
        j = match(t, t + lag * period, MATCH_FACTOR * period)
        ok = j >= 0
        pairs[lag - 1] = int(ok.sum())
        if pairs[lag - 1] >= 10:
            rho[lag - 1] = float(np.mean(yc[ok] * yc[j[ok]]) / var)
    return rho, pairs


def portmanteau(rho: np.ndarray, pairs: np.ndarray) -> tuple[float, float, int]:
    """Box-Pierce test that rho is the autocorrelation of white noise.

    Q = sum_l n_l rho_l^2 (n_l: pairs at lag l), chi^2 with L degrees of freedom under the
    hypothesis of whiteness. Returns Q, its p-value and L (finite lags only).
    """
    ok = np.isfinite(rho) & (pairs > 0)
    if not ok.any():
        return np.nan, np.nan, 0
    q = float(np.sum(pairs[ok] * rho[ok] ** 2))
    return q, float(chi2.sf(q, int(ok.sum()))), int(ok.sum())


def effective_size(n: int, rho: np.ndarray) -> float:
    """n / (1 + 2 sum rho_l): the independent samples a correlated sequence is worth (<= n)."""
    s = 1.0 + 2.0 * np.nansum(rho)
    return float(min(n, n / s)) if s > 0.0 else float(n)


def nis_interval(dof: int, n: int = 1, level: float = 0.95) -> tuple[float, float]:
    """Two-sided interval of the mean of n NIS values of a consistent filter: chi^2_(dof n) / n."""
    a = 0.5 * (1.0 - level)
    return float(chi2.ppf(a, dof * n) / n), float(chi2.ppf(1.0 - a, dof * n) / n)


# ── MPC logs ──────────────────────────────────────────────────────────────────

# FR3 kinematics: Franka's modified DH parameters (Craig), (a, d, alpha) per joint, then the
# flange (d = 0.107) and the hand's TCP 0.1034 m further along the flange z. This is the
# URDF's fr3_hand_tcp in fr3_link0: mpc_analysis.py checks it on every run against the
# plan's first point, which the MPC computed with Pinocchio on the URDF.
FR3_DH = ((0.0, 0.333, 0.0), (0.0, 0.0, -np.pi / 2), (0.0, 0.316, np.pi / 2), (0.0825, 0.0, np.pi / 2),
          (-0.0825, 0.384, -np.pi / 2), (0.0, 0.0, np.pi / 2), (0.088, 0.0, np.pi / 2))
FR3_TCP_FROM_LINK7 = 0.107 + 0.1034   # [m] along the last z axis


def fr3_tcp_positions(q: np.ndarray) -> np.ndarray:
    """TCP positions (N, 3) in fr3_link0 of the arm configurations q (N, 7); NaN rows stay NaN."""
    q = np.atleast_2d(np.asarray(q, dtype=float))
    T = np.tile(np.eye(4), (len(q), 1, 1))
    for i, (a, d, alpha) in enumerate(FR3_DH):
        ct, st, ca, sa = np.cos(q[:, i]), np.sin(q[:, i]), np.cos(alpha), np.sin(alpha)
        Ti = np.zeros_like(T)
        Ti[:, 0, 0], Ti[:, 0, 1], Ti[:, 0, 3] = ct, -st, a
        Ti[:, 1, 0], Ti[:, 1, 1], Ti[:, 1, 2], Ti[:, 1, 3] = st * ca, ct * ca, -sa, -sa * d
        Ti[:, 2, 0], Ti[:, 2, 1], Ti[:, 2, 2], Ti[:, 2, 3] = st * sa, ct * sa, ca, ca * d
        Ti[:, 3, 3] = 1.0
        T = T @ Ti
    return T[:, :3, 3] + FR3_TCP_FROM_LINK7 * T[:, :3, 2]


def read_log(run: Path, suffix: str, text_columns: tuple[str, ...] = ()) -> pd.DataFrame | None:
    """experiment_<suffix>.csv sorted by stamp, without duplicated stamps; None if missing or empty.

    Every column is numeric (anything else -> NaN) except text_columns, kept as strings.
    """
    path = run / f"experiment_{suffix}.csv"
    if not path.is_file() or path.stat().st_size == 0:
        return None
    try:
        df = pd.read_csv(path, on_bad_lines="skip", dtype={c: str for c in text_columns})
    except (pd.errors.EmptyDataError, pd.errors.ParserError, UnicodeDecodeError, OSError) as exc:
        warn(f"cannot read {path.name}: {exc}")
        return None
    if df.empty or "timestamp" not in df.columns:
        return None
    numeric = [c for c in df.columns if c not in text_columns]
    df[numeric] = df[numeric].apply(pd.to_numeric, errors="coerce").replace([np.inf, -np.inf], np.nan)
    for c in text_columns:
        if c in df.columns:
            df[c] = df[c].fillna("")
    df = df[df["timestamp"].notna()]
    if df.empty:
        return None
    df = df.sort_values("timestamp", kind="stable").drop_duplicates("timestamp", keep="last")
    return df.reset_index(drop=True)


def zoh(t_src: np.ndarray, y_src: np.ndarray, t_query: np.ndarray, max_age: float) -> np.ndarray:
    """y_src held from each stamp to the next (a command as the controller sees it).

    The value at each t_query is the last sample at or before it; NaN before the first
    sample or when that sample is older than max_age (the commands stopped).
    """
    y_src = np.asarray(y_src, dtype=float).reshape(len(t_src), -1)
    t_query = np.asarray(t_query, dtype=float)
    j = np.searchsorted(t_src, t_query, side="right") - 1
    ok = (j >= 0) & (t_query - np.asarray(t_src)[np.maximum(j, 0)] <= max_age)
    out = np.full((len(t_query), y_src.shape[1]), np.nan)
    out[ok] = y_src[j[ok]]
    return out


def on_grid(t: np.ndarray, y: np.ndarray, period: float) -> tuple[np.ndarray, np.ndarray]:
    """The samples placed on a uniform grid of this period from the first stamp.

    Returns (grid index of each sample, y on the grid with NaN where no sample landed).
    Lags on the grid are then plain index shifts, and a missing cycle never shifts one.
    """
    t = np.asarray(t, dtype=float)
    y = np.asarray(y, dtype=float).reshape(len(t), -1)
    k = np.round((t - t[0]) / period).astype(int)
    grid = np.full((int(k.max()) + 1, y.shape[1]), np.nan)
    grid[k] = y
    return k, grid


def lagged(x: np.ndarray, lag: int) -> np.ndarray:
    """x shifted by lag ≥ 0 samples (x_{k-lag} at k), NaN where it does not exist."""
    x = np.asarray(x, dtype=float)
    if lag == 0:
        return x.copy()
    out = np.full_like(x, np.nan)
    if lag < len(x):
        out[lag:] = x[:-lag]
    return out


def ar_fit(u: np.ndarray, order: int) -> np.ndarray:
    """AR coefficients a of u_k ≈ Σ_i a_i u_{k-i} (i = 1..order), least squares on the
    grid samples whose whole past window exists; zeros if too few of them."""
    X = np.column_stack([lagged(u, i) for i in range(1, order + 1)])
    ok = np.isfinite(X).all(axis=1) & np.isfinite(u)
    if ok.sum() < 5 * order:
        return np.zeros(order)
    a, *_ = np.linalg.lstsq(X[ok], u[ok], rcond=None)
    return a


def ar_filter(x: np.ndarray, a: np.ndarray) -> np.ndarray:
    """x_k − Σ_i a_i x_{k-i}: the prewhitening filter of ar_fit (NaN where the window is not whole)."""
    out = x.copy()
    for i, a_i in enumerate(a, start=1):
        out = out - a_i * lagged(x, i)
    return out


def cross_correlation_test(e: np.ndarray, u: np.ndarray, max_lag: int, order: int = 4):
    """Ljung's test that the residual e_k is uncorrelated with the inputs u_{k-τ}, τ = 0..max_lag.

    e, u: on the same grid (NaN = missing). In closed loop u is far from white, which makes
    the raw cross-correlation's ±1.96/√n band too narrow: both are first prewhitened with
    the AR model of u (Box-Jenkins), then r(τ) = corr(ẽ_k, ũ_{k-τ}) and
    S = Σ n_τ r(τ)² ~ χ²(max_lag + 1) if e does not depend on past inputs.
    Returns r (max_lag + 1,), pairs per lag, S and its p-value.
    """
    a = ar_fit(u, order)
    ew, uw = ar_filter(e, a), ar_filter(u, a)
    ew = ew - np.nanmean(ew)
    uw = uw - np.nanmean(uw)
    r, pairs = np.full(max_lag + 1, np.nan), np.zeros(max_lag + 1, dtype=int)
    for lag in range(max_lag + 1):
        ul = lagged(uw, lag)
        ok = np.isfinite(ew) & np.isfinite(ul)
        pairs[lag] = int(ok.sum())
        if pairs[lag] >= 10:
            se, su = np.std(ew[ok]), np.std(ul[ok])
            if se > 0 and su > 0:
                r[lag] = float(np.mean(ew[ok] * ul[ok]) / (se * su))
    ok = np.isfinite(r)
    if not ok.any():
        return r, pairs, np.nan, np.nan
    s = float(np.sum(pairs[ok] * r[ok] ** 2))
    return r, pairs, s, float(chi2.sf(s, int(ok.sum())))


def high_pass(y: np.ndarray, period: float, window_s: float) -> np.ndarray:
    """y minus its centred moving average over window_s: drops the task's slow motion."""
    n = max(int(round(window_s / period)), 1)
    trend = pd.Series(y).rolling(n, center=True, min_periods=max(n // 2, 1)).mean().to_numpy()
    return y - trend


def oscillation_index(y: np.ndarray, period: float, detrend_s: float = 1.0,
                      max_lag_s: float = 5.0) -> tuple[float, float, float]:
    """Regularity of an oscillation, from the zero crossings of the autocorrelation
    (Thornhill, Huang & Zhang, J. Process Control 2003).

    y: a uniform grid (NaN = missing), high-passed over detrend_s first. Each pair of
    alternate zero crossings of its ACF spans one period T_i; r = mean(T) / (3 std(T)).
    r > 1: a regular oscillation of period mean(T); noise and one-off events give r < 1.
    Returns (period [s], r, rms of the high-passed signal); NaN period and r with fewer
    than 4 period estimates within max_lag_s.
    """
    hp = high_pass(np.asarray(y, dtype=float), period, detrend_s)
    rms = float(np.sqrt(np.nanmean(hp ** 2))) if np.isfinite(hp).any() else np.nan
    x = np.nan_to_num(hp - np.nanmean(hp)) if np.isfinite(hp).any() else np.zeros(1)
    n, lags = len(x), min(int(max_lag_s / period), len(x) - 1)
    if lags < 4 or not np.any(x):
        return np.nan, np.nan, rms
    f = np.fft.rfft(x, 2 * n)
    acf = np.fft.irfft(f * np.conj(f))[:lags + 1]
    acf = acf / acf[0]
    s = np.sign(acf)
    idx = np.flatnonzero(s[:-1] * s[1:] < 0)
    crossings = idx + acf[idx] / (acf[idx] - acf[idx + 1])   # linear interpolation [lags]
    periods = (crossings[2:] - crossings[:-2]) * period
    if len(periods) < 4:
        return np.nan, np.nan, rms
    periods = periods[:10]
    sd = float(np.std(periods))
    return float(np.mean(periods)), (float(np.mean(periods) / (3.0 * sd)) if sd > 0 else np.inf), rms