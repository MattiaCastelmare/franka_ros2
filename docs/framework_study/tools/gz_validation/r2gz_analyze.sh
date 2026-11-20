#!/bin/bash
# R2 Gazebo static validation (C-078). Copy of the sealed refactor_r1_e3/scripts_h2/e3_analyze.sh with ONLY:
#   * run ids <NN>_s<seed>_<default|multi>; the gate's --mode is the EXPECTED executor: default -> single
#     (the shipped R2 default), multi -> multi. R01-R22 are unchanged;
#   * evidence root refactor_r2/05_gz_static, container root /home/user/r2gz_20261120;
#   * one added step, r2gz_extra (host, read-only: oscillation / smoothness / timing extras; recorded,
#     never part of the gate).
# Original header:
# E3 — offline analysis of ONE recorded run, then the pilot gate. No Gazebo, no launch.
# Read-only on the recorded cell: the S6B/S6B.1 scripts run UNCHANGED, and digest_cell (which writes
# into its run dir) runs on a COPY in the container scratch (F-6-10). Every step's exit code goes to
# analysis/steps.tsv; a failed step is reported and makes the gate STOP. The run directory is sealed
# (SHA256SUMS, write-protected) at the end, whatever the gate says.
#
# Usage:
#   e3_analyze.sh <run_id> [--include-aborted]
#   e3_analyze.sh --gate-only <cell_dir> <analysis_dir> <recon_dir> <multi|single> <new_out_dir>
#        (applies the gate to existing outputs, e.g. the S6B cell, writing only into <new_out_dir>)
# Exit codes: 0 gate CONTINUE | 3 gate STOP | 1 analysis or internal error | 2 usage
#
# The gate (pilot decisions of 2026-11-16) never reports "success": its outcomes are
#   STOP      a stop rule fired, an input is missing, or a step failed -> investigate before the next run
#   CONTINUE  no stop rule fired; the exploratory metrics are recorded WITHOUT acceptance thresholds
#
# scripts_h2 VERSION (H2): R01-R18 are those of scripts/e3_analyze.sh, unchanged. Added:
#   steps  h2_bag (container: per_link_distances, joint state at the birth, self-mask replay)
#          h2_metrics (host: placement record, pre-registered validity V1-V7, latencies, windows)
#   rules  R19 phase-aligned start recorded complete            (stop)
#          R20 pre-registered initial-condition validity V1-V7   (stop; the run is kept, nothing dropped)
#          R21 birth -> detection -> QP-row latencies, self-mask  (record)
#          R22 matched-window metrics present                    (stop on missing)
# E3_EXPECT_HEAD (40-hex) must be set, as for e3_run.sh: R05 checks the run's git provenance against it.

set -u -o pipefail
set -C

ROOT=$HOME/franka_baseline_evidence/refactor_r2/05_gz_static
S=$ROOT/scripts
CONTAINER=franka_ros2
CROOT=/home/user/r2gz_20261120
CS=$CROOT/scripts
LIB=/ros2_ws/install/franka_simulation_experiments/lib/franka_simulation_experiments

usage() { sed -n '15,19p' "$0" >&2; echo '  (R2 copy: run ids <NN>_s<seed>_<default|multi>)' >&2; exit 2; }
EXPECT_HEAD=${E3_EXPECT_HEAD:-}
[[ $EXPECT_HEAD =~ ^[0-9a-f]{40}$ ]] || { echo "e3_analyze: set E3_EXPECT_HEAD to the full 40-hex commit of H2" >&2; exit 2; }

gate() {   # all arguments go to the Python gate below
python3 -I - "$@" <<'PYEOF'
"""E3 pilot gate. Reads only; writes gate.json and gate.txt into --out (never overwrites)."""
import argparse
import glob
import json
import math
import os
import re
import sys

sys.dont_write_bytecode = True
ap = argparse.ArgumentParser()
ap.add_argument('--run-dir')
ap.add_argument('--cell-dir')
ap.add_argument('--analysis-dir')
ap.add_argument('--recon-dir')
ap.add_argument('--mode', required=True, choices=['multi', 'single'])
ap.add_argument('--out', required=True)
ap.add_argument('--expect-head', required=True)
a = ap.parse_args()

D_SAFE = 0.10      # fr3_control.yaml d_safe; also checked against the filter's own start-up line
EXACT = 1e-9       # equality of two evaluations of the same code on the same input (S6B.1 found 0.0)
QP_TARGET = 100.0  # conditions.yaml, condition full: qp_rate_hz
CLASS = {'multi': 'MultiThreadedExecutor', 'single': 'SingleThreadedExecutor'}
MISSING = object()


class Missing(Exception):
    pass


def load(path):
    try:
        with open(path) as f:
            return json.load(f)
    except (OSError, ValueError):
        return MISSING


def get(d, path, default=MISSING):
    for k in path.split('.'):
        if not isinstance(d, dict) or k not in d:
            return default
        d = d[k]
    return d


def need(src, path, name):
    v = MISSING if src is MISSING else get(src, path)
    if v is MISSING or v is None:
        raise Missing(f'{name}:{path}')
    return v


def fin(x):
    return isinstance(x, (int, float)) and not isinstance(x, bool) and math.isfinite(x)


RUN = a.run_dir
if RUN:
    cells = sorted(glob.glob(os.path.join(RUN, 'run', 'batch', 'cells', '*')))
    CELL = cells[0] if len(cells) == 1 else None
    ADIR = RDIR = os.path.join(RUN, 'analysis')
else:
    CELL, ADIR, RDIR = a.cell_dir, a.analysis_dir, a.recon_dir
if not CELL or not os.path.isdir(CELL):
    print(f'gate: cell directory not found ({CELL})', file=sys.stderr)
    sys.exit(1)

cr = load(os.path.join(CELL, 'cell_result.json'))
man = load(os.path.join(CELL, 'run_manifest_sim.json'))
A = load(os.path.join(ADIR, 's6b_analysis.json'))
H = load(os.path.join(ADIR, 's6b_host.json'))
R = load(os.path.join(RDIR, 's6b1_reconstruction.json'))
SUP = load(os.path.join(RDIR, 's6b1_supplementary.json'))
DG = load(os.path.join(ADIR, 'digest_cell.json'))
H2M = load(os.path.join(ADIR, 'h2_metrics.json'))
cl = get(cr, 'closed_loop') if cr is not MISSING else MISSING
try:
    with open(os.path.join(CELL, 'launch.log'), errors='replace') as f:
        LOG = f.read().splitlines()
except OSError:
    LOG = None

results = []


def rule(rid, text, kind='stop-rule'):
    def deco(fn):
        try:
            verdict, detail = fn()
        except Missing as e:
            verdict, detail = 'MISSING', f'required input absent: {e}'
        except Exception as e:  # noqa: BLE001 - every failure must be visible, never a PASS
            verdict, detail = 'ERROR', f'{type(e).__name__}: {e}'
        results.append(dict(id=rid, kind=kind, rule=text, verdict=verdict, detail=detail))
        return fn
    return deco


def need_log():
    if LOG is None:
        raise Missing('launch.log')
    return LOG


@rule('R01', 'run recorded completely (RUN_STATUS)')
def _():
    if not RUN:
        raise Missing('RUN_STATUS (gate-only mode has no run directory)')
    st = open(os.path.join(RUN, 'RUN_STATUS')).read().strip()
    return ('PASS' if st == 'COMPLETE_PENDING_ANALYSIS' else 'STOP'), st


@rule('R02', 'every analysis step exited 0, no traceback in the analysis consoles')
def _():
    if not RUN:
        raise Missing('analysis/steps.tsv (gate-only mode)')
    rows = [ln.split('\t') for ln in open(os.path.join(ADIR, 'steps.tsv')).read().splitlines()[1:] if ln]
    bad = [f'{n} rc={rc}' for n, rc in rows if rc != '0']
    tb = [os.path.basename(p) for p in glob.glob(os.path.join(ADIR, '*_console.txt'))
          if 'Traceback' in open(p, errors='replace').read()]
    ok = rows and not bad and not tb
    return ('PASS' if ok else 'STOP'), f'steps {len(rows)}; failed {bad}; consoles with traceback {tb}'


@rule('R03', 'cell completed: no timeout, no leftovers, runner rc 0, scorers ran')
def _():
    t, lo, rc = need(cr, 'timed_out', 'cell'), need(cr, 'leftovers', 'cell'), need(cr, 'returncode', 'cell')
    err = 'error' in cr
    ok = t is False and lo == [] and rc == 0 and not err
    return ('PASS' if ok else 'STOP'), f'timed_out={t} leftovers={lo} returncode={rc} scorer_error={err}'


@rule('R04', 'episode START and DONE logged; episode length >= scenario T')
def _():
    log = need_log()
    st = any('SIM episode START' in x for x in log)
    dn = any('SIM episode DONE' in x for x in log)
    t0, t1 = need(man, 'episode.t_start_sim', 'manifest'), need(man, 'episode.t_end_sim', 'manifest')
    T = need(man, 'scenario_resolved.T', 'manifest')
    ok = st and dn and (t1 - t0) >= T - 1e-6
    return ('PASS' if ok else 'STOP'), f'START={st} DONE={dn} episode={t1 - t0:.3f} s T={T}'


@rule('R05', 'arm identity: executor class, manifest, launch params and git provenance')
def _():
    log = need_log()
    seen = sorted(set(re.findall(r'executor_mode=(\w+) \((\w+)\)', '\n'.join(log))))
    mcp = need(man, 'condition.cbf_params', 'manifest').get('executor_mode', MISSING)
    sha = need(man, 'git.sha', 'manifest')
    dirty = get(man, 'git.dirty', None)
    lp = 'n/a (gate-only)'
    lp_ok = True
    if RUN:
        files = glob.glob(os.path.join(RUN, 'run', 'container_tmp', 'tmp__launch_params_*'))
        vals = []
        for p in files:
            m = re.findall(r'^\s*executor_mode:\s*(\S+)\s*$', open(p, errors='replace').read(), re.M)
            vals += m
        lp, lp_ok = vals, vals == [a.mode]
    ok = (seen == [(a.mode, CLASS[a.mode])] and mcp == a.mode and lp_ok and sha == a.expect_head)
    return ('PASS' if ok else 'STOP'), (f'start-up line {seen}; manifest executor_mode='
                                        f'{None if mcp is MISSING else mcp}; launch params {lp}; '
                                        f'git {sha[:12]} dirty={dirty}')


@rule('R06', 'controller active at episode start')
def _():
    c = need(cl, 'controller_active_at_start', 'closed_loop')
    m = need(man, 'episode.controller_active_at_start', 'manifest')
    return ('PASS' if c is True and m is True else 'STOP'), f'closed_loop={c} manifest={m}'


@rule('R07', 'no node died and no traceback before the shutdown sequence')
def _():
    log = need_log()
    sd = next((i for i, x in enumerate(log) if "sending signal 'SIGINT'" in x), len(log))
    dn = next((i for i, x in enumerate(log) if 'SIM episode DONE' in x), None)
    bad = [x[:140] for x in log[:sd] if 'process has died' in x or 'Traceback' in x]
    after = sum(1 for x in log[sd:] if 'process has died' in x or 'Traceback' in x)
    ok = not bad and dn is not None and dn < sd
    return ('PASS' if ok else 'STOP'), (f'before shutdown: {bad or "none"}; at shutdown (F-8-6 class, '
                                        f'recorded): {after}; DONE line before shutdown: {dn is not None and dn < sd}')


@rule('R08', 'reflex verdict PASS with zero trips (FAIL or NO_DATA = STOP)')
def _():
    v = need(cl, 'reflex_verdict', 'closed_loop')
    t, tw = need(cl, 'reflex_trips', 'closed_loop'), need(cl, 'reflex_trips_whole_run', 'closed_loop')
    return ('PASS' if v == 'PASS' and t == 0 and tw == 0 else 'STOP'), f'verdict={v} trips={t} whole_run={tw}'


@rule('R09', 'reflex rate clips under a PASS verdict', kind='record')
def _():
    return 'RECORD', f'rate_clips={need(cl, "rate_clips", "closed_loop")}'


@rule('R10', 'no controller fault (fault_ticks = 0 in the harness and in the per-tick status)')
def _():
    f1, f2 = need(cl, 'fault_ticks', 'closed_loop'), need(A, 'qp.fault_ticks', 's6b_analysis')
    sc, ll = get(cl, 'slew_conflict.ticks', None), get(cl, 'livelock_escapes', None)
    return ('PASS' if f1 == 0 and f2 == 0 else 'STOP'), (f'fault_ticks harness={f1} status={f2}; '
                                                         f'slew_conflict={sc} livelock_escapes={ll} (recorded)')


@rule('R11', 'no fallback (box-only or brake): STOP to investigate, not a collision verdict')
def _():
    fl = need(cl, 'fallback_levels', 'closed_loop')
    lv = need(A, 'qp.fallback_levels', 's6b_analysis')
    ok = fl.get('box-only') == 0 and fl.get('brake') == 0 and set(lv) <= {'0'}
    return ('PASS' if ok else 'STOP'), f'harness {fl}; per-tick status levels {lv}'


@rule('R12', 'no contact: harness contact false, reconstructed gap never < 0, bound >= 0')
def _():
    c = need(cl, 'contact', 'closed_loop')
    n0 = need(R, 'recon_clearance.n_samples_below_0', 's6b1_reconstruction')
    lb = need(SUP, 'sampling.continuous_lower_bound_m', 's6b1_supplementary')
    ok = c is False and n0 == 0 and fin(lb) and lb >= 0
    return ('PASS' if ok else 'STOP'), f'contact={c} recon samples<0={n0} continuous lower bound={lb}'


@rule('R13', 'clearance can be established (S6B.1 reconstruction consistent with the truth)')
def _():
    le = need(R, 'geometry.labels_equal', 's6b1_reconstruction')
    ea = need(R, 'check_A_same_q.max_abs_gap_err_m', 's6b1_reconstruction')
    eq = need(R, 'check_B_independent.max_abs_q_diff_rad', 's6b1_reconstruction')
    eb = need(SUP, 'episode_min_gap_abs_diff_excluding_placement_instant.max', 's6b1_supplementary')
    mono = need(R, 'joint_states.header_monotonic', 's6b1_reconstruction')
    lb = need(SUP, 'sampling.continuous_lower_bound_m', 's6b1_supplementary')
    mg = need(R, 'recon_clearance.min_gap', 's6b1_reconstruction')
    ok = (le is True and mono is True and fin(ea) and ea <= EXACT and fin(eq) and eq <= EXACT
          and fin(eb) and eb <= EXACT and fin(lb) and fin(mg))
    cov = (f'joint samples {get(R, "joint_states.n_episode", "?")}/{get(R, "joint_states.expected_at_1khz", "?")}, '
           f'max dt {get(R, "joint_states.max_dt_ms", "?")} ms; truth hole max '
           f'{get(A, "truth.hole_max_ms", "?")} ms; recon-only spans {len(get(SUP, "reconstruction_only_spans", []) or [])}')
    return ('PASS' if ok else 'STOP'), (f'labels_equal={le} header_monotonic={mono} gap_err(same q)={ea} '
                                        f'q_diff={eq} truth-vs-recon(excl. placement)={eb}; {cov}')


@rule('R14', 'clearance >= d_safe = 0.10 m, established with the conservative inter-sample bound')
def _():
    log = need_log()
    ds = sorted(set(float(x) for x in re.findall(r'd_safe=([0-9.]+) m\s+horizon', '\n'.join(log))))
    if ds != [D_SAFE]:
        return 'STOP', f'filter start-up d_safe {ds} != {D_SAFE}: configuration mismatch'
    mg = need(R, 'recon_clearance.min_gap', 's6b1_reconstruction')
    nb = need(R, 'recon_clearance.n_samples_below_dsafe', 's6b1_reconstruction')
    lb = need(SUP, 'sampling.continuous_lower_bound_m', 's6b1_supplementary')
    dip = need(SUP, 'sampling.max_intersample_dip_bound_m', 's6b1_supplementary')
    hm = get(cl, 'min_gap', None)
    detail = (f'reconstructed min {mg:.5f} m at t={get(R, "recon_clearance.t_min", "?")} '
              f'({get(R, "recon_clearance.cp_min", "?")}); inter-sample dip bound {dip:.6f} m; '
              f'continuous lower bound {lb:.5f} m; harness truth min {hm}')
    if mg < D_SAFE or nb > 0:
        return 'STOP', 'BELOW d_safe: ' + detail
    if lb < D_SAFE:
        return 'STOP', 'clearance >= d_safe CANNOT BE ESTABLISHED (bound below d_safe): ' + detail
    return 'PASS', detail


@rule('R15', 'QP-rate verdict known (RATE_FAIL is recorded, not a stop; UNKNOWN is a stop)')
def _():
    ok = get(cl, 'qp_rate_ok', MISSING)
    tgt = need(cl, 'qp_rate_target_hz', 'closed_loop')
    hz = need(cl, 'qp_rate_hz', 'closed_loop')
    if tgt != QP_TARGET:
        return 'STOP', f'qp_rate_target_hz={tgt} != {QP_TARGET}: configuration mismatch'
    if ok is True:
        return 'PASS', f'qp_rate_hz={hz:.2f} within +-5 % of {tgt}'
    if ok is False:
        return 'RECORD', f'RATE_FAIL: qp_rate_hz={hz:.2f} vs {tgt} (negative result, recorded, not a stop)'
    return 'STOP', f'qp_rate_ok={None if ok is MISSING else ok}: UNKNOWN verdict is not a pass'


@rule('R16', 'real-time factor (< 0.9 is the code flag; recorded, not a stop)', kind='record')
def _():
    r, r10 = need(cl, 'rtf', 'closed_loop'), get(cl, 'rtf_p10', None)
    rw = need(A, 'wall_map.episode_rtf', 's6b_analysis')
    return 'RECORD', f'rtf={r:.3f} rtf_p10={r10} episode_rtf(wall bag)={rw:.3f}' + (' [< 0.9]' if r < 0.9 else '')


REQUIRED = [
    (A, 's6b_analysis', ['qp.rate_per_sim_s', 'qp.missed', 'qp.tick_gap_ms', 'qp.gaps_ge_50ms',
                          'qp.gaps_ge_100ms', 'qp.solve_wall_ms', 'qp.ages_ms', 'rebuild.per_sim_s',
                          'rebuild.build_wall_ms', 'gap_vs_rebuild.p_long_after_new',
                          'gap_vs_rebuild.p_long_otherwise', 'family', 'ee.err_norm', 'ee.final_err',
                          'wall_map.episode_rtf', 'wall_map.tick_gap_wall_ms', 'truth.hole_max_ms']),
    (H, 's6b_host', ['cpu.process_pct_of_one_core', 'cpu.threads', 'cpu.samples_episode',
                     'cpu.host_busy_cores_equiv', 'truth_holes_gt10ms']),
    (R, 's6b1_reconstruction', ['recon_clearance.min_gap', 'recon_clearance.time_below_dsafe_s']),
    (SUP, 's6b1_supplementary', ['sampling.continuous_lower_bound_m']),
    (cl, 'closed_loop', ['slack_max', 'slack_max_any', 'capage_ms_p95', 'rows_rate_hz', 'pentagon.p95_m',
                         'jerk_p99']),
    (DG, 'digest_cell', ['qdd_projection', 'pentagon_tracking']),
]


@rule('R17', 'every required metric present (missing = STOP)')
def _():
    miss = [f'{n}:{p}' for src, n, ps in REQUIRED for p in ps
            if src is MISSING or get(src, p, MISSING) in (MISSING, None)]
    extra = []
    if H is not MISSING:
        if not (get(H, 'cpu.samples_episode', 0) or 0) > 0:
            extra.append('no CPU samples in the episode')
        if not any(k.startswith('cbf_safety_filter[') for k in (get(H, 'cpu.process_pct_of_one_core', {}) or {})):
            extra.append('cbf_safety_filter absent from the CPU samples')
    if A is not MISSING and not (get(A, 'wall_map.wall_status_msgs', 0) or 0) > 0:
        extra.append('wall bag has no /NS_1/cbf_status messages')
    if DG is not MISSING:
        for k in ('experiment_summary_rc', 'smoothness_report_rc'):
            if get(DG, k, None) not in (0, None):
                extra.append(f'digest {k}={get(DG, k)}')
    bad = miss + extra
    return ('PASS' if not bad else 'STOP'), (f'missing {miss}; problems {extra}' if bad else 'all present')


@rule('R18', 'run integrity (git, install, repository untouched)')
def _():
    if not RUN:
        raise Missing('postflight (gate-only mode)')
    pf = os.path.join(RUN, 'postflight')
    pre = os.path.join(RUN, 'preflight')
    same = all(open(os.path.join(pre, f'{x}_before.txt')).read() == open(os.path.join(pf, f'{x}_after.txt')).read()
               for x in ('git_status', 'git_stash'))
    newer = open(os.path.join(pf, 'repo_files_newer_than_marker.txt')).read().strip()
    inst = 'INSTALLED == SOURCE' in open(os.path.join(pf, 'check_installed.txt')).read()
    ok = same and not newer and inst
    return ('PASS' if ok else 'STOP'), f'git unchanged={same}; repo files written={bool(newer)}; installed==source={inst}'


@rule('R19', 'H2 phase-aligned start: phase mode, started after a qualified clean cycle, record complete')
def _():
    em = need(man, 'episode_start.mode', 'manifest')
    st = need(man, 'episode.start.status', 'manifest')
    dec = get(man, 'episode.start.qualifier.decision', None)
    pl = get(man, 'episode.start.placement', None)
    if em != 'phase' or st != 'started' or not dec or not pl:
        return 'STOP', (f'episode_start={em} status={st} fail_reason={get(man, "episode.start.fail_reason", None)} '
                        f'decision={bool(dec)} placement={bool(pl)}')
    ok = dec['window_cycles'] >= 1.0 and 0.25 - 1e-9 <= dec['lead_s'] <= 0.55
    return ('PASS' if ok else 'STOP'), (f'announced at {dec["t_announce"]:.3f} for {dec["t_start"]:.4f} '
                                        f'(lead {dec["lead_s"]:.3f} s), {dec["window_cycles"]:.3f} clean cycles '
                                        f'from {dec["window_t0"]:.3f}, max |e| {1e3 * dec["window_max_err_m"]:.1f} mm, '
                                        f'rate x{dec["rate_over_nominal"]:.4f}; controller active since '
                                        f'{dec["controller_active_since"]}; HARD ticks before announce '
                                        f'{dec["hard_reset_ticks_before_announce"]}; supervisor phase error '
                                        f'{pl["phase_error_cycles"]:+.5f} cyc')


@rule('R20', 'pre-registered initial-condition validity V1-V7 (INVALID = STOP; data kept, every rule still reported)')
def _():
    v = need(H2M, 'validity', 'h2_metrics')
    crit = v.get('criteria') or []
    bad = [f"{c['id']} {c['verdict']}" for c in crit if c['verdict'] != 'PASS']
    det = '; '.join(f"{c['id']} {c['verdict']}" for c in crit) or v.get('reason', '')
    return ('PASS' if v.get('verdict') == 'VALID' and not bad and len(crit) == 7 else 'STOP'), \
        f"{v.get('verdict')}: {det}"


@rule('R21', 'birth -> first non-empty frame -> first obstacle row in the QP; self-mask at the birth', kind='record')
def _():
    L = need(H2M, 'latency', 'h2_metrics')
    pl = need(H2M, 'placement', 'h2_metrics')
    mk = pl.get('mask_at_birth') or {}
    return 'RECORD', (f"frame capture +{L['birth_to_first_nonempty_frame_capture_s']} s "
                      f"(receipt +{L['birth_to_first_nonempty_frame_receipt_s']} s, empty frames "
                      f"{L['empty_frames_after_birth']}), first QP obstacle row +{L['birth_to_first_qp_obstacle_row_s']} s; "
                      f"replay at birth: visible {mk.get('visible')} / deleted {mk.get('deleted_by_self_mask')} / "
                      f"hidden {mk.get('hidden_by_arm')} of {mk.get('ball_px')} px")


@rule('R22', 'matched-window metrics present (after t0+2 s and the six curve-phase windows)')
def _():
    W = need(H2M, 'windows', 'h2_metrics')
    want = ['after_t0+2'] + [f'phase_{k}/6' for k in range(6)]
    miss = [k for k in want if not W.get(k)]
    keys = ['qp_ticks_per_sim_s', 'tick_gap_ms', 'n_rows', 'rebuilds_per_sim_s', 'build_wall_ms', 'solve_wall_ms',
            'rtf', 'filter_cpu_pct_one_core', 'min_clearance_m', 'tracking_err_m']
    holes = [f'{k}.{m}' for k in want if W.get(k) for m in keys if W[k].get(m) is None]
    return ('PASS' if not miss and not holes else 'STOP'), f'missing windows {miss}; missing metrics {holes}'


# ---------------------------------------------------------------- exploratory metrics (no thresholds)
def pick(src, path, fmt='{}'):
    v = get(src, path, None) if src is not MISSING else None
    try:
        return fmt.format(v) if v is not None else 'n/a'
    except (ValueError, TypeError):
        return str(v)


filt_cpu = 'n/a'
filt_thr = 'n/a'
if H is not MISSING:
    pp = get(H, 'cpu.process_pct_of_one_core', {}) or {}
    filt_cpu = ', '.join(f'{k}: {v}' for k, v in pp.items() if k.startswith('cbf_safety_filter[')) or 'n/a'
    th = get(H, 'cpu.threads', {}) or {}
    filt_thr = '; '.join(f'{len(v)} threads {sorted(v.values(), reverse=True)}' for k, v in th.items()
                         if k.startswith('cbf_safety_filter[')) or 'n/a'
EXPLORATORY = [
    ('1 QP ticks per sim s', pick(A, 'qp.rate_per_sim_s', '{:.2f}')),
    ('1 missed 10 ms slots (episode)', pick(A, 'qp.missed', '{:.0f}')),
    ('2 tick gap sim ms p50/p95/p99', '/'.join(pick(A, f'qp.tick_gap_ms.{p}', '{:.1f}') for p in ('p50', 'p95', 'p99'))),
    ('2 tick gap sim ms max', pick(A, 'qp.tick_gap_ms.max', '{:.1f}')),
    ('2 gaps >= 50 / >= 100 ms', f"{pick(A, 'qp.gaps_ge_50ms')} / {pick(A, 'qp.gaps_ge_100ms')}"),
    ('2 tick gap wall ms p50/p95', '/'.join(pick(A, f'wall_map.tick_gap_wall_ms.{p}', '{:.1f}') for p in ('p50', 'p95'))),
    ('3 rebuilds per sim s', pick(A, 'rebuild.per_sim_s', '{:.1f}')),
    ('3 rebuild wall ms p50/p95', '/'.join(pick(A, f'rebuild.build_wall_ms.{p}', '{:.2f}') for p in ('p50', 'p95'))),
    ('3 solve wall ms p50/p95', '/'.join(pick(A, f'qp.solve_wall_ms.{p}', '{:.2f}') for p in ('p50', 'p95'))),
    ('3 P(late tick | fresh rebuild) / otherwise', f"{pick(A, 'gap_vs_rebuild.p_long_after_new', '{:.3f}')} / "
                                                   f"{pick(A, 'gap_vs_rebuild.p_long_otherwise', '{:.4f}')}"),
    ('4 filter process CPU % of one core', filt_cpu),
    ('4 filter threads (% each)', filt_thr),
    ('4 host busy cores (equiv.)', pick(H, 'cpu.host_busy_cores_equiv', '{:.1f}')),
    ('4 RTF harness / wall bag', f"{pick(cl, 'rtf', '{:.3f}')} / {pick(A, 'wall_map.episode_rtf', '{:.3f}')}"),
    ('5 clearance reconstructed min / lower bound m', f"{pick(R, 'recon_clearance.min_gap', '{:.5f}')} / "
                                                       f"{pick(SUP, 'sampling.continuous_lower_bound_m', '{:.5f}')}"),
    ('5 harness truth min gap m / truth hole max ms', f"{pick(cl, 'min_gap', '{:.5f}')} / {pick(A, 'truth.hole_max_ms', '{:.0f}')}"),
    ('6 EE error m p50/p95, final', '/'.join(pick(A, f'ee.err_norm.{p}', '{:.4f}') for p in ('p50', 'p95'))
     + f", final {pick(A, 'ee.final_err', '{:.4f}')}"),
    ('6 pentagon p95 / rms m (harness)', f"{pick(cl, 'pentagon.p95_m', '{:.4f}')} / {pick(cl, 'pentagon.rms_m', '{:.4f}')}"),
    ('7 slack max any / obs / cap', f"{pick(cl, 'slack_max_any', '{:.3f}')} / {pick(cl, 'slack_max.obs', '{:.3f}')} / "
                                     f"{pick(cl, 'slack_max.cap', '{:.3f}')}"),
    ('7 obstacle-slack ticks', pick(A, 'family.obs.slack.ticks_pos')),
    ('7 fallback levels / fault ticks', f"{pick(cl, 'fallback_levels')} / {pick(cl, 'fault_ticks')}"),
    ('7 reflex verdict / trips / rate clips', f"{pick(cl, 'reflex_verdict')} / {pick(cl, 'reflex_trips')} / {pick(cl, 'rate_clips')}"),
    ('8 cell ok (NOT acceptance, F-6-6)', pick(cr, 'ok')),
    ('- perception capture age p95 ms / rows Hz', f"{pick(cl, 'capage_ms_p95', '{:.1f}')} / {pick(cl, 'rows_rate_hz', '{:.1f}')}"),
    ('- jerk p99', pick(cl, 'jerk_p99', '{:.1f}')),
    ('H2 birth phase (bag) / error cyc', f"{pick(H2M, 'placement.phase_at_birth', '{:.5f}')} / "
                                         f"{pick(H2M, 'placement.phase_error_cycles', '{:+.5f}')}"),
    ('H2 initial clearance m (cp) / g_min', f"{pick(H2M, 'placement.initial_clearance_m', '{:.4f}')} "
                                            f"({pick(H2M, 'placement.initial_clearance_cp')}) / {pick(H2M, 'prereg.g_min', '{:.4f}')}"),
    ('H2 |e| at birth / max last cycle m', f"{pick(H2M, 'placement.tracking_err_at_birth_m', '{:.4f}')} / "
                                           f"{pick(H2M, 'placement.tracking_err_max_last_cycle_m', '{:.4f}')}"),
    ('H2 latency frame / QP row s', f"{pick(H2M, 'latency.birth_to_first_nonempty_frame_capture_s', '{:.3f}')} / "
                                    f"{pick(H2M, 'latency.birth_to_first_qp_obstacle_row_s', '{:.3f}')}"),
] + [(f'W {k}: ticks/s, gap p99/max ms, rows p50, rebuild/s, build/solve p50 ms, RTF, CPU %, min gap, |e| p95',
      (lambda w: 'n/a' if not w else
       f"{w['qp_ticks_per_sim_s']:.1f}, {w['tick_gap_ms']['p99']:.1f}/{w['tick_gap_ms']['max']:.1f}, "
       f"{w['n_rows']['p50']:.0f}, {w['rebuilds_per_sim_s']:.1f}, {w['build_wall_ms']['p50']:.1f}/"
       f"{w['solve_wall_ms']['p50']:.2f}, {w['rtf'] if w['rtf'] is None else round(w['rtf'], 3)}, "
       f"{w['filter_cpu_pct_one_core'] if w['filter_cpu_pct_one_core'] is None else round(w['filter_cpu_pct_one_core'], 1)}, "
       f"{w['min_clearance_m']:.3f}, {(w['tracking_err_m'] or {}).get('p95', float('nan')):.4f}")(
          get(H2M, 'windows', {}).get(k) if H2M is not MISSING else None))
   for k in (['after_t0+2'] + [f'phase_{k}/6' for k in range(6)] + ['transient_t0..t0+1.5 (not matched)'])]

fatal = [r for r in results if r['verdict'] in ('STOP', 'MISSING', 'ERROR')]
overall = 'STOP' if fatal else 'CONTINUE'
os.makedirs(a.out, exist_ok=True)
doc = dict(overall=overall, mode=a.mode, cell=CELL, run_dir=RUN, rules=results,
           exploratory={k: v for k, v in EXPLORATORY},
           note=('CONTINUE means no pilot stop rule fired. It is NOT an acceptance: T1, T6-T10 have no '
                 'agreed thresholds and the exploratory metrics are reported without them.'))
with open(os.path.join(a.out, 'gate.json'), 'x') as f:
    json.dump(doc, f, indent=1, default=str)
lines = [f'E3 GATE {overall}  ({a.mode})  {CELL}', '']
lines += [f"{r['id']} {r['verdict']:8s} {r['rule']}\n             {r['detail']}" for r in results]
lines += ['', 'Exploratory metrics (no thresholds; numbers refer to the metric list of the plan):']
lines += [f'  {k:46s} {v}' for k, v in EXPLORATORY]
lines += ['', doc['note']]
txt = '\n'.join(lines) + '\n'
with open(os.path.join(a.out, 'gate.txt'), 'x') as f:
    f.write(txt)
print(txt)
sys.exit(3 if fatal else 0)
PYEOF
}

# ------------------------------------------------------------------ gate-only mode
if [ "${1:-}" = "--gate-only" ]; then
  [ $# = 6 ] || usage
  case $5 in multi|single) ;; *) usage;; esac
  [ -e "$6" ] && { echo "e3_analyze: output dir $6 exists" >&2; exit 1; }
  mkdir -p "$6" || exit 1
  gate --cell-dir "$2" --analysis-dir "$3" --recon-dir "$4" --mode "$5" --out "$6" --expect-head "$EXPECT_HEAD"
  exit $?
fi

# ------------------------------------------------------------------ full analysis of one run
ID=${1:-}; [ -n "$ID" ] || usage
INCLUDE_ABORTED=0; [ "${2:-}" = "--include-aborted" ] && INCLUDE_ABORTED=1
[[ $ID =~ ^[0-9]{2}_s[012]_(default|multi)$ ]] || usage
ARM=${ID##*_}
case $ARM in default) MODE=single;; multi) MODE=multi;; esac
D=$ROOT/runs/$ID
RUN=$CROOT/$ID
[ -d "$D" ] || { echo "e3_analyze: no run $D" >&2; exit 1; }
exec 9>>"$ROOT/.e3_lock" || exit 1
flock -n 9 || { echo "e3_analyze: another E3 script holds the lock" >&2; exit 1; }
ST=$(cat "$D/RUN_STATUS" 2>/dev/null)
if [ "$ST" != COMPLETE_PENDING_ANALYSIS ] && [ "$INCLUDE_ABORTED" != 1 ]; then
  echo "e3_analyze: RUN_STATUS is '$ST'; refusing (use --include-aborted to analyse it anyway; the gate will STOP)" >&2
  exit 1
fi
[ -e "$D/analysis" ] && { echo "e3_analyze: $D/analysis exists (already analysed?)" >&2; exit 1; }
[ -w "$D" ] || { echo "e3_analyze: $D is sealed (not writable)" >&2; exit 1; }
mapfile -t CELLS < <(cd "$D/run/batch/cells" 2>/dev/null && ls -d -- */ 2>/dev/null | tr -d /)
[ "${#CELLS[@]}" = 1 ] || { echo "e3_analyze: expected exactly one cell in $D/run/batch/cells, found ${#CELLS[@]}" >&2; exit 1; }
CELL=${CELLS[0]}
C=$RUN/batch/cells/$CELL
docker exec "$CONTAINER" test -d "$C" || { echo "e3_analyze: container cell $C missing" >&2; exit 1; }
docker exec "$CONTAINER" test ! -e "$RUN/analysis" || { echo "e3_analyze: $RUN/analysis exists" >&2; exit 1; }
docker exec "$CONTAINER" test ! -e "$RUN/digest_copy" || { echo "e3_analyze: $RUN/digest_copy exists" >&2; exit 1; }

mkdir "$D/analysis" || exit 1
STEPS=$D/analysis/steps.tsv
printf 'step\trc\n' > "$STEPS"
FAILED=0
step() {   # step <name> <command...>; stdout+stderr -> analysis/<name>_console.txt
  local name=$1; shift
  "$@" > "$D/analysis/${name}_console.txt" 2>&1
  local rc=$?
  printf '%s\t%s\n' "$name" "$rc" >> "$STEPS"
  if [ "$rc" != 0 ]; then FAILED=1; echo "ANALYSIS STEP FAILED: $name (rc $rc) — see analysis/${name}_console.txt" >&2
  else echo "step $name: ok" >&2; fi
}
X() { docker exec -w /tmp -e PYTHONDONTWRITEBYTECODE=1 "$CONTAINER" bash -c \
        "source /ros2_ws/install/setup.bash >/dev/null 2>&1; $1"; }
stage_scripts() {
  docker exec "$CONTAINER" mkdir -p "$CS" "$RUN/analysis" || return 1
  local f h c
  for f in s6b_analyze.py s6b1_reconstruct_clearance.py e3_h2_bag.py; do
    docker cp "$S/$f" "$CONTAINER:$CS/$f" || return 1
    h=$(sha256sum < "$S/$f" | cut -c1-64); c=$(docker exec "$CONTAINER" sha256sum "$CS/$f" | cut -c1-64)
    echo "$f host=$h container=$c"; [ "$h" = "$c" ] || return 1
  done
}
step stage_scripts stage_scripts
step s6b_analyze X "python3 -B $CS/s6b_analyze.py $C $RUN/wallbag $RUN/analysis"
step s6b1_reconstruct X "python3 -B $CS/s6b1_reconstruct_clearance.py $C $RUN/analysis"
step h2_bag X "python3 -B $CS/e3_h2_bag.py $C $RUN/analysis $RUN/analysis"
step digest_cell X "cp -a $C $RUN/digest_copy && python3 -B -c \"import sys, json; sys.path.insert(0, '$LIB'); \
from simexp import runner; json.dump(runner.digest_cell('$RUN/digest_copy'), open('$RUN/analysis/digest_cell.json', 'x'), indent=1, default=str)\""
step copy_back docker cp "$CONTAINER:$RUN/analysis/." "$D/analysis/"
step s6b_host python3 -I "$S/s6b_host.py" "$D" "$D/analysis/s6b_host.json"
step s6b1_supplementary python3 -I "$S/s6b1_supplementary.py" "$D/run/batch/cells/$CELL" \
     "$D/analysis/s6b1_reconstruction.npz" "$D/analysis/s6b1_supplementary.json"
step h2_metrics python3 -I "$S/e3_h2_metrics.py" "$D" "$D/analysis" "$D/analysis/h2_metrics.json"
step r2gz_extra python3 -I "$S/r2gz_extra.py" "$D" "$D/analysis/r2gz_extra.json"

gate --run-dir "$D" --mode "$MODE" --out "$D/analysis" --expect-head "$EXPECT_HEAD" > /dev/null
GRC=$?
case $GRC in
  0) [ "$FAILED" = 0 ] && AS=GATE_CONTINUE || AS=ANALYSIS_ERROR ;;
  3) AS=GATE_STOP ;;
  *) AS=ANALYSIS_ERROR ;;
esac
printf '%s\n' "$AS" > "$D/ANALYSIS_STATUS"
[ -f "$D/analysis/gate.txt" ] && cat "$D/analysis/gate.txt"

(cd "$D" && find . -type f ! -name SHA256SUMS -print0 | sort -z | xargs -0 sha256sum > SHA256SUMS) \
  && chmod -R a-w "$D" && echo "sealed $D ($(wc -l < "$D/SHA256SUMS") files)" \
  || { echo "e3_analyze: SEALING FAILED for $D" >&2; AS=ANALYSIS_ERROR; }

echo "ANALYSIS_STATUS=$AS"
case $AS in GATE_CONTINUE) exit 0;; GATE_STOP) exit 3;; *) exit 1;; esac
