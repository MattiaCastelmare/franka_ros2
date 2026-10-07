"""Record a presentation video of a policy running in FrankaCBF-v0.

Two side-by-side panels, laid out like the lab recordings
(combined_rgb_blur_depth_1920.mp4: 1920x542 @ 29.97 fps):

  left   3D view — measured EE trajectory (blue), target (green), obstacle
         (red) with its d_safe shell, task/episode text.
  right  top view — one segment per control point to the obstacle surface,
         coloured by the CBF margin (green d > d_safe, orange 0 < d <= d_safe,
         red d <= 0), plus a d_min(t) strip.

Frames are taken on the SIMULATED clock, so playback is real time whatever the
render speed. Every episode runs the full --episode-s: reaching the target
does NOT end it (terminate_on_success is overridden for the video only), so
the arm is seen holding the target while the obstacle keeps sweeping. The
policy observes no time, so running past the 5 s training horizon is
in-distribution for it. A collision still ends the episode. Needs a GL context. Inside the container
both EGL and GLFW work but land on the Intel iGPU (Mesa, ~15 ms/view): the
image ships libEGL_nvidia but no glvnd vendor file for it. Registering one puts
EGL on the RTX 4070 (~1.4 ms/view):

    printf '{"file_format_version":"1.0.0","ICD":{"library_path":"libEGL_nvidia.so.0"}}' \\
        > /tmp/10_nvidia.json

    cd /ros2_ws/src && PYTHONPATH=/ros2_ws/src MUJOCO_GL=egl \\
        __EGL_VENDOR_LIBRARY_FILENAMES=/tmp/10_nvidia.json \\
        python3 -m franka_sim.scripts.record_video \\
            --model franka_sim/models/sac_v9/best_model.zip --episodes 10 \\
            --out franka_sim/runs/videos/sac_v9_raw.mp4

cv2 writes mp4v; re-encode to H.264 on the host for players/slides:

    ffmpeg -i sac_v9_raw.mp4 -c:v libx264 -pix_fmt yuv420p -crf 20 sac_v9.mp4
"""

from __future__ import annotations

import argparse
import copy
import os

import cv2
import mujoco
import numpy as np
import yaml

from franka_sim.envs.franka_cbf_env import FrankaCBFEnv
from franka_sim.scripts.evaluate_policy import _DEFAULT_CONFIG, _load_policy

W, H = 1920, 542
PW = W // 2
FPS = 30000 / 1001
HOLD_S = 1.5                         # freeze on the outcome banner
TITLE_S = 3.0                        # opening card
SUMMARY_S = 8.0                      # closing card with the aggregate metrics
# [rad/s^2] ‖q̈_safe − q̈_nom‖ above which the shield counts as ACTIVE. cbf_n_c
# is useless for this: the six obstacle rows and the workspace rows are
# always in the QP, so n_c > 0 on every tick.
ACTIVE_TOL = 0.05

# BGR, matching the lab overlay colours.
C_WHITE = (255, 255, 255)
C_BLUE = (255, 140, 40)
C_GREEN = (60, 220, 60)
C_ORANGE = (0, 165, 255)
C_RED = (40, 40, 230)
C_GREY = (150, 150, 150)
C_MAGENTA = (255, 0, 255)


def _rgba(bgr, a=1.0):
    b, g, r = bgr
    return np.array([r / 255, g / 255, b / 255, a], np.float32)


def _margin_colour(d, d_safe):
    if d <= 0.0:
        return C_RED
    return C_ORANGE if d <= d_safe else C_GREEN


# ── Scene decorations (user geoms appended after update_scene) ─────────────────

def _add_geom(scn, gtype, size, pos, rgba):
    if scn.ngeom >= scn.maxgeom:
        return
    g = scn.geoms[scn.ngeom]
    mujoco.mjv_initGeom(g, gtype, np.asarray(size, np.float64),
                        np.asarray(pos, np.float64), np.eye(3).flatten(), rgba)
    scn.ngeom += 1


def _add_segment(scn, p0, p1, width, rgba):
    if scn.ngeom >= scn.maxgeom or np.linalg.norm(p1 - p0) < 1e-6:
        return
    g = scn.geoms[scn.ngeom]
    mujoco.mjv_initGeom(g, mujoco.mjtGeom.mjGEOM_CAPSULE, np.zeros(3),
                        np.zeros(3), np.eye(3).flatten(), rgba)
    mujoco.mjv_connector(g, mujoco.mjtGeom.mjGEOM_CAPSULE, width,
                         np.asarray(p0, np.float64), np.asarray(p1, np.float64))
    scn.ngeom += 1


def _decorate(scn, env, trail, d_safe, segments):
    p_obs = env.data.mocap_pos[env._obs_mocap]
    # d_safe shell around the obstacle: where the barrier starts to bite.
    _add_geom(scn, mujoco.mjtGeom.mjGEOM_SPHERE, [env.r_obs + d_safe] * 3,
              p_obs, _rgba(C_ORANGE, 0.10))
    for k in range(1, len(trail)):
        _add_segment(scn, trail[k - 1], trail[k], 0.004, _rgba(C_BLUE))
    if not segments:
        return
    for bid, r_cp in zip(env._cp_body, env._cp_radius):
        p_cp = env.data.xpos[bid]
        diff = p_cp - p_obs
        dist = float(np.linalg.norm(diff))
        if dist < 1e-6:
            continue
        n_hat = diff / dist
        d = dist - env.r_obs - r_cp
        col = _margin_colour(d, d_safe)
        _add_geom(scn, mujoco.mjtGeom.mjGEOM_SPHERE, [0.012] * 3, p_cp,
                  _rgba(C_ORANGE))
        _add_segment(scn, p_cp, p_obs + n_hat * env.r_obs, 0.003, _rgba(col))


# ── 2D overlays ────────────────────────────────────────────────────────────────

FONT = cv2.FONT_HERSHEY_SIMPLEX


def _label_block(img, lines, x=6, y=6, scale=0.55, thick=2):
    """Text on a black box — the lab-video style.

    Each line is ``(text, colour)`` or a list of such segments, so a legend can
    draw every entry in the colour it describes.
    """
    for line in lines:
        segs = line if isinstance(line, list) else [line]
        sizes = [cv2.getTextSize(t, FONT, scale, thick) for t, _ in segs]
        tw = sum(sz[0][0] for sz in sizes)
        th = max(sz[0][1] for sz in sizes)
        base = max(sz[1] for sz in sizes)
        cv2.rectangle(img, (x, y), (x + tw + 8, y + th + base + 6), (0, 0, 0), -1)
        cx = x + 4
        for (text, col), ((w, _), _) in zip(segs, sizes):
            cv2.putText(img, text, (cx, y + th + 3), FONT, scale, col, thick,
                        cv2.LINE_AA)
            cx += w
        y += th + base + 7


def _banner(img, text, col):
    scale, thick = 1.6, 4
    (tw, th), _ = cv2.getTextSize(text, FONT, scale, thick)
    x, y = (img.shape[1] - tw) // 2, (img.shape[0] + th) // 2
    cv2.rectangle(img, (x - 20, y - th - 20), (x + tw + 20, y + 20), (0, 0, 0), -1)
    cv2.putText(img, text, (x, y), FONT, scale, col, thick, cv2.LINE_AA)


def _plot_strip(img, d_hist, active_hist, d_safe, t_max, y_max=0.5):
    """d_min(t) over the whole episode, CBF-active ticks shaded."""
    h, x0, x1 = 120, 50, img.shape[1] - 12
    y1 = img.shape[0] - 10
    y0 = y1 - h
    roi = img[y0 - 22:y1 + 4, x0 - 44:x1 + 6]
    roi[:] = (roi * 0.35).astype(np.uint8)

    def ty(v):
        return int(y1 - np.clip(v / y_max, 0, 1) * h)

    n = len(d_hist)
    xs = x0 + (np.arange(n) / max(1, t_max - 1) * (x1 - x0)).astype(int)
    for k in range(n):
        if active_hist[k]:
            cv2.line(img, (xs[k], y0), (xs[k], y1), (110, 40, 110), 2)
    cv2.line(img, (x0, ty(d_safe)), (x1, ty(d_safe)), C_ORANGE, 1, cv2.LINE_AA)
    cv2.line(img, (x0, ty(0.0)), (x1, ty(0.0)), C_RED, 1, cv2.LINE_AA)
    if n > 1:
        pts = np.stack([xs, [ty(v) for v in d_hist]], 1).astype(np.int32)
        cv2.polylines(img, [pts], False, C_GREEN, 2, cv2.LINE_AA)
    cv2.rectangle(img, (x0, y0), (x1, y1), C_GREY, 1)
    cv2.putText(img, 'd_min(t) [m]   shaded = shield active', (x0, y0 - 6), FONT,
                0.45, C_WHITE, 1, cv2.LINE_AA)
    for v in (0.0, d_safe, y_max):
        cv2.putText(img, f'{v:.2f}', (x0 - 42, ty(v) + 4), FONT, 0.4, C_WHITE,
                    1, cv2.LINE_AA)


def _card(writer, lines, seconds, scale=0.8, right=()):
    """Full-frame text card held for ``seconds``; returns the frames written.

    ``right`` is an optional second column starting at mid-frame.
    """
    img = np.zeros((H, W, 3), np.uint8)
    for x, column in ((40, lines), (PW + 20, right)):
        y = 20
        for text, col in column:
            (_, th), base = cv2.getTextSize(text or ' ', FONT, scale, 2)
            cv2.putText(img, text, (x, y + th), FONT, scale, col, 2, cv2.LINE_AA)
            y += th + base + 8
    n = int(round(seconds * FPS))
    for _ in range(n):
        writer.write(img)
    return n


# ── Main loop ──────────────────────────────────────────────────────────────────

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--model', required=True,
                    help='.onnx, SB3 .zip, or "zero" / "random"')
    ap.add_argument('--config', default=None,
                    help='config.yaml (default: the one frozen next to the model)')
    ap.add_argument('--episodes', type=int, default=10)
    ap.add_argument('--episode-s', type=float, default=10.0,
                    help='fixed episode length [s]; success does not end it')
    ap.add_argument('--seed', type=int, default=12345)
    ap.add_argument('--seeds', default=None,
                    help='comma-separated seed list (overrides --seed/--episodes), '
                         'e.g. to replay benchmark episodes')
    ap.add_argument('--set', action='append', default=[], metavar='SECTION.KEY=VALUE',
                    help='config override, any depth (repeatable), as in '
                         'runs/eval_all/test_traces.py')
    ap.add_argument('--held', action='store_true',
                    help='benchmark outcome: SUCCESS only if the EE is within '
                         'target_tol at the END (held), else NOT HELD')
    ap.add_argument('--label', default=None,
                    help='policy name shown in the video (default: model dir)')
    ap.add_argument('--out', default='franka_sim/runs/videos/policy_raw.mp4')
    ap.add_argument('--title-s', type=float, default=TITLE_S,
                    help='seconds of the opening card (0 = no card, e.g. when the video gets its own cards)')
    ap.add_argument('--summary-s', type=float, default=SUMMARY_S,
                    help='seconds of the closing summary card (0 = no card)')
    ap.add_argument('--pad', action='store_true',
                    help='hold the outcome frame until --episode-s, so an episode that ends early '
                         '(collision) lasts as long as a full one: videos stacked side by side stay in sync')
    ap.add_argument('--intro', action='append', default=[],
                    help='extra line for the title card (repeatable), e.g. '
                         'the 40-seed benchmark numbers')
    args = ap.parse_args()

    cfg = args.config
    if cfg is None:
        frozen = os.path.join(os.path.dirname(os.path.abspath(args.model)),
                              'config.yaml')
        cfg = frozen if os.path.isfile(frozen) else _DEFAULT_CONFIG
    label = args.label or os.path.basename(
        os.path.dirname(os.path.abspath(args.model)))

    predict, kind = _load_policy(args.model)
    with open(cfg) as f:
        cfg_d = copy.deepcopy(yaml.safe_load(f))
    rate = float(cfg_d.get('env', {}).get('control_rate_hz', 100.0))
    cfg_d.setdefault('env', {})['max_episode_steps'] = int(round(args.episode_s * rate))
    cfg_d.setdefault('reward', {})['terminate_on_success'] = False
    for kv in args.set:
        k, v = kv.split('=', 1); *path, key = k.split('.'); d = cfg_d
        for p in path:
            d = d.setdefault(p, {})
        d[key] = yaml.safe_load(v)
    seeds = ([int(x) for x in args.seeds.split(',')] if args.seeds
             else [args.seed + ep for ep in range(args.episodes)])
    args.episodes = len(seeds)
    cfg_d['reward']['terminate_on_collision'] = True
    env = FrankaCBFEnv(config=cfg_d)
    d_safe = float(env.cfg['cbf']['d_safe'])
    shield = ('HOCBF shield' if env.cbf_obstacle_enabled
              else 'NO obstacle CBF (joint/ws box only)')
    print(f'policy={args.model} ({kind})  config={cfg}  episodes={args.episodes}')

    env.model.vis.global_.offwidth = max(env.model.vis.global_.offwidth, PW)
    env.model.vis.global_.offheight = max(env.model.vis.global_.offheight, H)
    renderer = mujoco.Renderer(env.model, H, PW, max_geom=20000)

    cam3d = mujoco.MjvCamera()
    cam3d.type = mujoco.mjtCamera.mjCAMERA_FREE
    cam3d.lookat[:] = [0.35, 0.0, 0.45]
    cam3d.distance, cam3d.azimuth, cam3d.elevation = 1.45, 225.0, -18.0

    camtop = mujoco.MjvCamera()
    camtop.type = mujoco.mjtCamera.mjCAMERA_FREE
    camtop.lookat[:] = [0.35, 0.0, 0.30]
    camtop.distance, camtop.azimuth, camtop.elevation = 1.65, 180.0, -89.0

    os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
    writer = cv2.VideoWriter(args.out, cv2.VideoWriter_fourcc(*'mp4v'), FPS, (W, H))
    if not writer.isOpened():
        raise RuntimeError(f'cannot open {args.out} for writing')

    n_frames = 0
    tally = {'SUCCESS': 0, 'COLLISION': 0, 'TIMEOUT': 0, 'NOT HELD': 0}
    stats = []
    title = [(f'{label}  -  {args.episodes} test episodes', C_WHITE),
             (f'safety layer: {shield}   d_safe = {d_safe:.2f} m', C_WHITE),
             (f'each episode runs {args.episode_s:.0f} s; success = EE within '
              f'{1000 * env.target_tol:.0f} mm of the target', C_GREY),
             ('collision (d_min < 0) ends the episode', C_GREY)]
    title += [(line, C_GREEN) for line in args.intro]
    n_frames += _card(writer, title, args.title_s)
    for ep in range(args.episodes):
        obs, _ = env.reset(seed=seeds[ep])
        trail = [env._ee_pos()]
        d_hist, act_hist, int_hist = [], [], []
        floor_steps = 0
        info = {'dist': float(np.linalg.norm(trail[0] - env._target)),
                'd_min': 99.0, 'cbf_intervention': 0.0}
        step, next_frame, outcome, t_reach = 0, 0.0, None, None
        n_frames_ep0 = n_frames

        def frame():
            renderer.update_scene(env.data, cam3d)
            _decorate(renderer.scene, env, trail, d_safe, segments=False)
            left = cv2.cvtColor(renderer.render(), cv2.COLOR_RGB2BGR)
            renderer.update_scene(env.data, camtop)
            _decorate(renderer.scene, env, trail, d_safe, segments=True)
            right = cv2.cvtColor(renderer.render(), cv2.COLOR_RGB2BGR)

            t = step * env.dt
            reach = (('target reached at t = %.2f s' % t_reach, C_GREEN)
                     if t_reach is not None else ('target not reached yet', C_GREY))
            _label_block(left, [
                (f'SIMULATION  {label} + {shield}', C_WHITE),
                (f'episode {ep + 1}/{args.episodes} (seed {seeds[ep]})   t = {t:4.2f} s', C_WHITE),
                (f'EE-target error {1000 * info["dist"]:.0f} mm', C_WHITE),
                reach,
                (f'so far: {tally["SUCCESS"]} success  {tally["COLLISION"]} collision'
                 + (f'  {tally["NOT HELD"]} not held' if args.held else
                    f'  {tally["TIMEOUT"]} timeout'), C_WHITE),
                ((f'FLOOR CONTACT  ({floor_steps} steps)', C_RED) if floor_steps
                 else ('no floor contact', C_GREEN)),
                ('blue = measured EE', C_BLUE),
                [('green = target', C_GREEN), ('   red = obstacle', C_RED),
                 ('   orange shell = d_safe', C_ORANGE)],
            ])
            d = info['d_min']
            d_txt = f'{d:.3f} m' if d < 50 else '--'
            _label_block(right, [
                ('top view', C_WHITE),
                (f'd_min {d_txt}   (d_safe {d_safe:.2f} m)',
                 _margin_colour(d, d_safe)),
                (f'shield {"ACTIVE" if info["cbf_intervention"] > ACTIVE_TOL else "idle"}'
                 f'   |qdd_safe - qdd_nom| {info["cbf_intervention"]:.2f} rad/s^2',
                 C_MAGENTA if info['cbf_intervention'] > ACTIVE_TOL else C_WHITE),
                [('link-obstacle lines:  ', C_WHITE), ('green d > d_safe', C_GREEN),
                 ('   orange d < d_safe', C_ORANGE), ('   red contact', C_RED)],
            ], scale=0.5)
            _plot_strip(right, d_hist, act_hist, d_safe, env.max_steps)
            cv2.line(right, (0, 0), (0, H), (0, 0, 0), 3)
            return np.hstack([left, right])

        while True:
            if step * env.dt >= next_frame - 1e-9:
                writer.write(frame())
                n_frames += 1
                next_frame += 1.0 / FPS
            if outcome is not None:
                break
            obs, _, term, trunc, info = env.step(predict(obs))
            step += 1
            trail.append(env._ee_pos())
            d_hist.append(info['d_min'])
            act_hist.append(info['cbf_intervention'] > ACTIVE_TOL)
            int_hist.append(info['cbf_intervention'])
            floor_steps += bool(info.get('floor_contact', False))
            if info['success'] and t_reach is None:
                t_reach = step * env.dt
            if info['collision']:
                outcome = 'COLLISION'
            elif term or trunc:
                if args.held:
                    outcome = 'SUCCESS' if info['dist'] < env.target_tol else 'NOT HELD'
                else:
                    outcome = 'SUCCESS' if t_reach is not None else 'TIMEOUT'

        # The loop above wrote the final state only if it fell on a frame tick.
        last = frame()
        col = {'SUCCESS': C_GREEN, 'COLLISION': C_RED, 'TIMEOUT': C_ORANGE, 'NOT HELD': C_ORANGE}[outcome]
        st = {'outcome': outcome, 't_reach': t_reach, 'err': info['dist'],
              'min_d': min(d_hist), 'active': float(np.mean(act_hist)),
              'interv': float(np.mean(int_hist)), 't_end': step * env.dt,
              'floor': floor_steps}
        stats.append(st)
        _banner(last[:, :PW], outcome, col)
        _label_block(last[:, :PW], [
            ('reached at %.2f s' % t_reach if t_reach is not None
             else 'target never reached', C_GREEN if t_reach else C_GREY),
            (f'final error {1000 * st["err"]:.0f} mm', C_WHITE),
            (f'min distance {1000 * st["min_d"]:.0f} mm',
             _margin_colour(st['min_d'], d_safe)),
            (f'shield active {100 * st["active"]:.0f}% of steps', C_WHITE),
            ((f'floor contact {floor_steps} steps', C_RED) if floor_steps
             else ('no floor contact', C_GREEN)),
        ], x=6, y=H - 146)
        n_hold = int(round(HOLD_S * FPS))
        if args.pad:   # frames a full episode would have written in the loop: ticks 0 .. episode_s
            n_hold += max(0, int(np.floor(args.episode_s * FPS + 1e-6)) + 1 - (n_frames - n_frames_ep0))
        for _ in range(n_hold):
            writer.write(last)
            n_frames += 1
        tally[outcome] += 1
        reached = f'{t_reach:.2f} s' if t_reach is not None else '-'
        print(f'episode {ep + 1}: {outcome:9s}  steps={step}  reached={reached}  '
              f'final err={info["dist"]:.3f} m  min d={min(d_hist):.3f} m')

    n = len(stats)
    reach_t = [s['t_reach'] for s in stats if s['t_reach'] is not None]
    summary = [
        (f'{label}  -  summary of {n} test episodes', C_WHITE),
        (f'success {tally["SUCCESS"]}/{n}    collision {tally["COLLISION"]}/{n}'
         + (f'    not held {tally["NOT HELD"]}/{n}' if args.held else
            f'    timeout {tally["TIMEOUT"]}/{n}'),
         C_RED if tally['COLLISION'] else C_GREEN),
        ('mean time to reach target: ' +
         (f'{np.mean(reach_t):.2f} s' if reach_t else 'never reached'), C_WHITE),
        (f'final EE-target error: mean {1000 * np.mean([s["err"] for s in stats]):.0f} mm'
         f'   median {1000 * np.median([s["err"] for s in stats]):.0f} mm', C_WHITE),
        (f'min distance to obstacle: worst {1000 * min(s["min_d"] for s in stats):.0f} mm'
         f'   mean {1000 * np.mean([s["min_d"] for s in stats]):.0f} mm'
         f'   (d_safe {1000 * d_safe:.0f} mm)', C_WHITE),
        (f'shield active {100 * np.mean([s["active"] for s in stats]):.0f}% of steps'
         f'   mean |qdd_safe - qdd_nom| {np.mean([s["interv"] for s in stats]):.2f} rad/s^2',
         C_WHITE),
        (f'episodes touching the floor: {sum(s["floor"] > 0 for s in stats)}/{n}',
         C_RED if any(s['floor'] for s in stats) else C_GREEN),
        ('', C_WHITE),
    ]
    summary += [(line, C_GREEN) for line in args.intro]
    per_ep = [('per episode', C_WHITE)]
    for k, s in enumerate(stats):
        col = {'SUCCESS': C_GREEN, 'COLLISION': C_RED, 'TIMEOUT': C_ORANGE, 'NOT HELD': C_ORANGE}[s['outcome']]
        rt = f'{s["t_reach"]:.2f} s' if s['t_reach'] is not None else '  -  '
        per_ep.append((f'ep {k + 1:2d}  {s["outcome"]:9s}  reached {rt:>7s}  '
                        f'err {1000 * s["err"]:4.0f} mm  min d {1000 * s["min_d"]:4.0f} mm'
                        + ('  FLOOR' if s['floor'] else ''),
                        col))
    n_frames += _card(writer, summary, args.summary_s, scale=0.55, right=per_ep)
    print(f'summary: {tally}  mean err {np.mean([s["err"] for s in stats]):.3f} m')

    writer.release()
    renderer.close()
    env.close()
    print(f'{n_frames} frames ({n_frames / FPS:.1f} s) -> {args.out}   {tally}')


if __name__ == '__main__':
    main()
