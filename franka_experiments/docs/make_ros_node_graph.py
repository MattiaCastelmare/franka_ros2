#!/usr/bin/env python3
"""Hand-laid-out SVG of the torque_control_stack ROS 2 node graph.

Usage: python3 make_ros_node_graph.py ros_node_graph.svg
PNG: google-chrome --headless=new --force-device-scale-factor=2 --window-size=2240,1280 \\
       --screenshot=raw.png ros_node_graph.svg  &&  convert raw.png -crop 4480x2192+0+0 +repage ros_node_graph.png
"""
import sys
from xml.sax.saxutils import escape

W, H = 2240, 1096
SANS = "Inter, 'DejaVu Sans', Arial, sans-serif"
MONO = "'DejaVu Sans Mono', Menlo, Consolas, monospace"

LAYER = {  # stroke, fill, title colour
    'perc':  ('#0f766e', '#e8f5f2', '#0b5d57'),
    'motion': ('#1d4ed8', '#eaf0fd', '#1e3a8a'),
    'safety': ('#c2410c', '#fdf0e8', '#9a3412'),
    'dyn':   ('#7c3aed', '#f3eefe', '#5b21b6'),
    'rt':    ('#334155', '#edf0f4', '#1e293b'),
    'hw':    ('#111827', '#e2e5ea', '#111827'),
    'supp':  ('#64748b', '#ffffff', '#334155'),
    'mon':   ('#4d7c0f', '#f1f7e8', '#3f6212'),
}
EDGE = {  # colour, width, dash
    'cmd':  ('#111827', 3.4, None),
    'ref':  ('#111827', 1.8, None),
    'dist': ('#0f766e', 2.6, None),
    'js':   ('#475569', 2.8, None),
    'stat': ('#c2410c', 2.0, '7 5'),
    'alt':  ('#1d4ed8', 2.4, '7 5'),
    'distalt': ('#0f766e', 2.0, '7 5'),
}
BAND_FILL = '#f6f7f9'

out = []
def add(s): out.append(s)

def text(x, y, s, size=13, color='#374151', font=SANS, anchor='start',
         weight='normal', style='normal', halo=None, ls=None):
    extra = ''
    if halo:
        extra += (f' stroke="{halo}" stroke-width="5" paint-order="stroke"'
                  ' stroke-linejoin="round"')
    if ls:
        extra += f' letter-spacing="{ls}"'
    add(f'<text x="{x}" y="{y}" font-family="{font}" font-size="{size}" '
        f'fill="{color}" text-anchor="{anchor}" font-weight="{weight}" '
        f'font-style="{style}"{extra}>{escape(s)}</text>')

def node(x, y, w, h, layer, title, lines=(), pkg=None, dashed=False,
         title_size=17, title_font=MONO):
    stroke, fill, tcol = LAYER[layer]
    dash = ' stroke-dasharray="8 5"' if dashed else ''
    add(f'<rect x="{x}" y="{y}" width="{w}" height="{h}" rx="10" '
        f'fill="{fill}" stroke="{stroke}" stroke-width="2"{dash}/>')
    # coloured accent bar on the left edge
    if not dashed:
        add(f'<rect x="{x}" y="{y+10}" width="5" height="{h-20}" rx="2.5" fill="{stroke}"/>')
    text(x + 16, y + (28 if h > 40 else h / 2 + 5), title, size=title_size, color=tcol, font=title_font,
         weight='bold')
    for i, ln in enumerate(lines):
        text(x + 16, y + 50 + 19 * i, ln, size=13.2, color='#374151')
    if pkg:
        text(x + 16, y + h - 11, pkg, size=11.5, color='#7b8494', style='italic')

def edge(pts, kind, start_arrow=False, end_arrow=True):
    col, wdt, dash = EDGE[kind]
    d = 'M' + ' L'.join(f'{px},{py}' for px, py in pts)
    da = f' stroke-dasharray="{dash}"' if dash else ''
    ms = f' marker-start="url(#a-{kind})"' if start_arrow else ''
    me = f' marker-end="url(#a-{kind})"' if end_arrow else ''
    add(f'<path d="{d}" fill="none" stroke="{col}" stroke-width="{wdt}"'
        f' stroke-linejoin="round"{da}{ms}{me}/>')

def topic(x, y, s, anchor='middle', size=12.5, color='#111827', halo=BAND_FILL):
    text(x, y, s, size=size, color=color, font=MONO, anchor=anchor, halo=halo,
         weight='bold')

def note(x, y, s, anchor='middle', size=11.5, color='#6b7280', halo=BAND_FILL):
    text(x, y, s, size=size, color=color, anchor=anchor, halo=halo)

def band(y, h, label):
    add(f'<rect x="30" y="{y}" width="{W-60}" height="{h}" rx="14" fill="{BAND_FILL}"/>')
    text(48, y + 22, label, size=12, color='#94a3b8', weight='bold', ls='2')

# ── document ────────────────────────────────────────────────────────────────
add(f'<svg xmlns="http://www.w3.org/2000/svg" width="{W}" height="{H}" '
    f'viewBox="0 0 {W} {H}">')
add('<defs>')
for k, (col, wdt, _) in EDGE.items():
    add(f'<marker id="a-{k}" markerUnits="userSpaceOnUse" markerWidth="14" '
        f'markerHeight="12" refX="12" refY="6" orient="auto-start-reverse">'
        f'<path d="M0,0 L14,6 L0,12 z" fill="{col}"/></marker>')
add('</defs>')
add(f'<rect width="{W}" height="{H}" fill="#ffffff"/>')

text(50, 62, 'franka_experiments — torque control stack: ROS 2 nodes & topics',
     size=31, color='#0f172a', weight='bold')
text(50, 94, 'ros2 launch franka_experiments torque_control_stack.launch.py   ·   '
     'FR3 + RealSense D455   ·   topic names from config/fr3_control.yaml',
     size=16, color='#64748b')

band(124, 176, 'PERCEPTION  ·  SUPERVISION')
band(312, 402, 'CONTROL LOOP')
band(732, 186, 'BRINGUP  ·  MONITORING  ·  LOGGING')

# ── perception band ─────────────────────────────────────────────────────────
node(50, 163, 300, 100, 'perc', 'realsense2_camera',
     ['Intel RealSense D455 (scene camera)', 'depth 848×480 @ 90 fps'],
     pkg='realsense2_camera (driver)')
node(560, 160, 370, 106, 'perc', 'real_time_distance',
     ['robot self-mask from the URDF meshes',
      'distance from every control point to obstacles',
      'obstacle clustering + Kalman tracking'])
node(1120, 160, 340, 106, 'safety', 'iso_safety_monitor',
     ['independent 2nd channel: latches a stop',
      'also reads franka/joint_states, cbf_status',
      'optional · iso_monitor_enabled:=true'], dashed=True)

# camera -> real_time_distance
edge([(350, 210), (560, 210)], 'dist')
topic(455, 200, '…/depth/image_rect_raw', size=11.5)
topic(455, 230, '…/depth/camera_info', size=11.5)
# real_time_distance -> iso monitor
edge([(930, 212), (1120, 212)], 'dist')
topic(1025, 202, '/cbf/per_link_distances', size=11.5)

# ── control band ────────────────────────────────────────────────────────────
node(50, 342, 320, 100, 'motion', 'rl_policy_commander',
     ['ONNX Safe-RL policy (SAC, from franka_sim)',
      'reads joint_states, distances, cbf_status',
      'alternative · motion_source:=rl'], dashed=True)
# "exactly one" pill
add('<rect x="105" y="450" width="210" height="24" rx="12" fill="#ffffff" '
    'stroke="#1d4ed8" stroke-width="1.2"/>')
text(210, 467, 'exactly one  ·  motion_source', size=12, color='#1d4ed8',
     anchor='middle', weight='bold')
node(50, 484, 320, 104, 'motion', 'pentagon_qddot_commander',
     ['path P(s) + timing law on phase s',
      'Cartesian PD → nominal q̈   ·   100 Hz',
      'governor slows s near obstacles'])

node(560, 392, 370, 123, 'safety', 'cbf_safety_filter',
     ['CBF-QP on joint accelerations (OSQP)',
      'min ‖q̈ − q̈_nom‖²  s.t.  barrier rows',
      'obstacles · self-collision · joint limits',
      'rows 50 Hz · QP 300 Hz (SCHED_FIFO)'])

node(1120, 418, 300, 104, 'dyn', 'qddot_to_torque',
     ['inverse dynamics (Pinocchio)',
      'τ = M(q)·q̈ + C(q,q̇)·q̇',
      'one τ per q̈_safe message'])

# ros2_control_node container
add('<rect x="1560" y="322" width="440" height="350" rx="12" fill="#ffffff" '
    'stroke="#334155" stroke-width="2.4"/>')
text(1578, 350, 'ros2_control_node', size=17, color='#1e293b', font=MONO, weight='bold')
text(1578, 368, 'controller_manager (started by franka_bringup)', size=11.5,
     color='#7b8494', style='italic')
node(1580, 378, 400, 120, 'rt', 'rt_torque_controller',
     ['1 kHz RT loop · SCHED_FIFO on an isolated core',
      'τ = τ_ff + Kp(q_des − q) + Kd(q̇_des − q̇)',
      'holds position if torque_cmd goes stale'],
     pkg='franka_rt_controllers')
node(1580, 520, 400, 54, 'rt', 'FrankaHardwareInterface', [],
     title_size=15)
text(1596, 563, 'franka_hardware · libfranka', size=11.5, color='#7b8494', style='italic')
node(1580, 586, 400, 34, 'rt', 'joint_state_broadcaster', [], title_size=14)
node(1580, 628, 400, 34, 'rt', 'franka_robot_state_broadcaster', [], title_size=14)
node(2060, 486, 150, 120, 'hw', 'Franka FR3',
     ['7-DoF arm', 'robot controller', '+ firmware'],
     title_size=18, title_font=SANS)

# commanders -> cbf (q̈_nom)
edge([(370, 400), (420, 400), (420, 452)], 'alt', end_arrow=False)
edge([(370, 536), (420, 536), (420, 452), (560, 452)], 'cmd')
topic(490, 442, '/NS_1/qddot_nom')
note(490, 471, 'q̈_nom ∈ ℝ⁷')

# cbf -> commander: status for the phase governor
edge([(640, 515), (640, 566), (370, 566)], 'stat')
topic(505, 586, '/NS_1/cbf_status', size=11.5, color='#9a3412')

# real_time_distance -> cbf (and -> rl commander)
edge([(745, 266), (745, 392)], 'dist')
topic(735, 316, '/cbf/per_link_distances', anchor='end', size=11.5, halo='#ffffff')
note(735, 332, 'MultiLinkDistance · best-effort', anchor='end', halo=BAND_FILL)
edge([(560, 245), (480, 245), (480, 365), (370, 365)], 'distalt')

# iso monitor -> cbf
edge([(1160, 266), (1160, 330), (860, 330), (860, 392)], 'stat')
topic(1010, 322, '/NS_1/iso_safety', size=11.5, color='#9a3412')
note(1010, 346, '(also → pentagon_qddot_commander)', size=11)

# cbf -> qddot_to_torque
edge([(930, 470), (1120, 470)], 'cmd')
topic(1025, 460, '/NS_1/qddot_safe')
note(1025, 489, 'q̈_safe ∈ ℝ⁷')

# cbf -> rt controller: feed-forward reference
edge([(900, 392), (900, 372), (1540, 372), (1540, 398), (1580, 398)], 'ref')
note(1320, 364, '/NS_1/qddot_safe  →  integrated into q_des, q̇_des for the PD',
     size=11.5, color='#374151')

# qddot_to_torque -> rt controller
edge([(1420, 470), (1580, 470)], 'cmd')
topic(1490, 460, '/NS_1/torque_cmd')
note(1490, 489, 'τ ∈ ℝ⁷  [N·m]')

# rt controller -> hardware interface
edge([(1700, 498), (1700, 520)], 'ref')
note(1712, 513, 'effort command interface', anchor='start', halo='#ffffff', size=11)

# hardware interface <-> robot
edge([(1980, 547), (2060, 547)], 'cmd', start_arrow=True)
note(2020, 537, 'FCI 1 kHz', halo='#ffffff', size=11.5, color='#111827')

# joint-state bus
edge([(1580, 603), (1540, 603), (1540, 692), (250, 692)], 'js', end_arrow=False)
edge([(1270, 692), (1270, 522)], 'js')
edge([(840, 692), (840, 515)], 'js')
edge([(250, 692), (250, 588)], 'js')
edge([(340, 692), (340, 768)], 'js')
topic(1055, 684, '/NS_1/franka/joint_states', color='#334155')
note(1055, 706, 'q, q̇ straight from the broadcaster · 1 kHz', size=11.5)

# ── bringup / monitoring band ───────────────────────────────────────────────
CW, CH, CY = 336, 130, 768
cx = [50, 412, 774, 1136, 1498, 1860]
node(cx[0], CY, CW, CH, 'supp', 'joint_state_publisher',
     ['re-stamps arm + gripper states',
      '→ /NS_1/joint_states @ 30 Hz',
      'for TF, move_group, rl commander, logger'],
     pkg='franka_bringup')
node(cx[1], CY, CW, CH, 'supp', 'robot_state_publisher',
     ['URDF + /NS_1/joint_states → /tf',
      'static TF: world → base,',
      'base → camera_link (calibration)'],
     pkg='franka_bringup · tf2_ros')
node(cx[2], CY, CW, CH, 'supp', 'move_group',
     ['MoveIt 2 (OMPL): launched by default,',
      'not used by the control loop',
      '+ franka_gripper_node (gripper)'],
     pkg='franka_fr3_moveit_config · franka_gripper', dashed=True)
node(cx[3], CY, CW, CH, 'mon', 'experiment_logger',
     ['subscribes ~12 topics: states, distances,',
      'q̈_nom, q̈_safe, τ, cbf_status, iso_safety,',
      'torque_saturation → CSV run log @ 100 Hz'])
node(cx[4], CY, CW, CH, 'mon', 'trajectory_overlay',
     ['/NS_1/ee_desired (red) vs ee_actual (blue),',
      'both from the commander, drawn as trails',
      'on the D455 colour image',
      '→ /NS_1/trajectory_overlay'])
node(cx[5], CY, CW, CH, 'mon', 'optional extras',
     ['iso_evidence_logger (ISO evidence)',
      'trajectory_visualization + rviz2',
      'ros2 bag record  ·  image_republisher'],
     pkg='switched on in config/launch_defaults.yaml', dashed=True,
     title_font=SANS)

# ── legend ──────────────────────────────────────────────────────────────────
LY = 958
text(50, LY, 'LEGEND', size=12, color='#94a3b8', weight='bold', ls='2')
sw = [('perc', 'perception'), ('motion', 'motion generation'),
      ('safety', 'safety (CBF / ISO)'), ('dyn', 'dynamics'),
      ('rt', 'RT control (ros2_control)'), ('hw', 'robot hardware'),
      ('supp', 'bringup / support'), ('mon', 'monitoring / logging')]
x = 50
for k, lbl in sw:
    stroke, fill, _ = LAYER[k]
    add(f'<rect x="{x}" y="{LY+16}" width="26" height="18" rx="4" fill="{fill}" '
        f'stroke="{stroke}" stroke-width="2"/>')
    text(x + 34, LY + 30, lbl, size=13.5, color='#374151')
    x += 34 + len(lbl) * 7.6 + 34

ly2 = LY + 66
items = [('cmd', 'command chain  q̈_nom → q̈_safe → τ'),
         ('dist', 'obstacle distances'),
         ('js', 'joint-state feedback (1 kHz)'),
         ('stat', 'status / supervision (not a command)'),
         ('ref', 'secondary signal')]
x = 50
for k, lbl in items:
    edge([(x, ly2 - 5), (x + 48, ly2 - 5)], k)
    text(x + 60, ly2, lbl, size=13.5, color='#374151')
    x += 60 + len(lbl) * 7.4 + 40
add(f'<rect x="{x}" y="{ly2-17}" width="34" height="22" rx="5" fill="#ffffff" '
    'stroke="#64748b" stroke-width="2" stroke-dasharray="6 4"/>')
text(x + 44, ly2, 'optional / alternative node', size=13.5, color='#374151')

text(50, ly2 + 44, 'Nodes without a package tag are executables of franka_experiments.  '
     'All arrays are std_msgs/Float64MultiArray over the 7 arm joints; '
     '/cbf/per_link_distances is franka_msgs/MultiLinkDistance.',
     size=12.5, color='#6b7280')

add('</svg>')
open(sys.argv[1], 'w').write('\n'.join(out))
