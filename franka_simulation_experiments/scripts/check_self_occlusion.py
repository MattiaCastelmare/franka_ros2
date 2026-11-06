"""N2's premise, tested directly against ground truth.

When a close track disappears from /cbf/per_link_distances, was the ball
actually behind the arm as seen from the DIAG camera? Ground-truth ball centre
from truth.npz, arm geometry from the SAME control-point definition the filter
uses (CBFKinematics), camera origin from camera_extrinsics_diag.yaml.
"""
import json, os, sys
import numpy as np

RUN = sys.argv[1]
CAM = np.array([1.993009, 0.032024, 1.080394])

sys.path.insert(0, '/ros2_ws/src/franka_simulation_experiments/scripts')
from simexp import analysis as A
from franka_msgs.msg import MultiLinkDistance
from sensor_msgs.msg import JointState
import pinocchio as pin
from franka_experiments.utils.kinematics import CBFKinematics, build_urdf_no_hand
from franka_experiments.utils.config import load_config_file
from ament_index_python.packages import get_package_share_directory
from franka_experiments.utils.cbf_state_rows import in_self_shadow

z = dict(np.load(RUN + '/truth.npz', allow_pickle=False))
man = json.load(open(RUN + '/run_manifest_sim.json'))
t0, t1 = man['episode']['t_start_sim'], man['episode']['t_end_sim']
tt = z['t']; P = z['p'][:, 0, :]; r_obs = float(z['obstacle_radius'][0])

b = A.read_bag_topics(RUN + '/bag', {'/cbf/per_link_distances': MultiLinkDistance,
                                     '/NS_1/franka/joint_states': JointState})
dist = b['/cbf/per_link_distances']; js = b['/NS_1/franka/joint_states']
ORDER = ['fr3_joint%d' % i for i in range(1, 8)]
jt = np.array([t for t, _ in js])
jq = np.array([[m.position[m.name.index(n)] for n in ORDER] for _, m in js])

cfg = load_config_file(os.path.join(
    get_package_share_directory('franka_experiments'), 'config', 'fr3_complete.yaml'))
kin = CBFKinematics(pin.buildModelFromUrdf(build_urdf_no_hand()))
kin.configure_control_points(cfg['robot'], cfg['distance'])
labels = sorted(kin._control_points)

prev, close_prev = set(), {}
events = []
for t, m in dist:
    cur = {}
    for l in m.links:
        if l.valid and l.track_id:
            cur[l.track_id] = min(cur.get(l.track_id, 9.9), l.distance)
    for k in prev - set(cur):
        if close_prev.get(k, 9.9) <= 0.25:
            events.append((t, k, close_prev[k]))
    prev = set(cur); close_prev.update(cur)

print(f'{len(events)} disappearances of a track that was within 25 cm\n')
hid = 0
for t, tid, d_last in events:
    q = jq[np.argmin(np.abs(jt - t))]
    kin.update(q, np.zeros(7))
    poly = [kin.control_point_position(l) for l in labels]
    ball = P[np.argmin(np.abs(tt - t))]
    shadow = in_self_shadow(CAM, ball, poly, 0.08)
    # how far the camera->ball ray passes from the nearest control point
    d = ball - CAM; L = np.linalg.norm(d); u = d / L
    between = [np.linalg.norm((p - CAM) - np.dot(p - CAM, u) * u)
               for p in poly if 0 < np.dot(p - CAM, u) < L]
    miss = min(between) if between else float('nan')
    hid += bool(shadow)
    where = (f'ray passes {100*miss:5.1f} cm from the arm' if between
             else 'no arm point between camera and ball')
    verdict = 'HIDDEN by the arm' if shadow else 'not explained by the arm'
    print(f'  t={t:6.2f}  last gap {100*d_last:5.1f} cm  {where:42s} -> {verdict}')
print(f'\nexplained by self-occlusion: {hid}/{len(events)}')

# ── The predicate the perception chain actually implements ──────────────────
# fr3_complete.yaml `mask:`: the exclusion mask is the robot's DILATED
# SILHOUETTE (12 px body / 24 px EE at full res), gated in depth by
# bias+tol = 0.16 m. So a track dies when it is (a) within the halo of an arm
# point IN IMAGE SPACE and (b) within 0.16 m of that point's range -- whether
# or not the arm is in front of it. The halo converts to an angle via fx, so
# this needs only the camera ORIGIN, not its orientation.
FX = 431.5
HALO_BODY_PX, HALO_EE_PX = 12.0, 24.0
DEPTH_TOL = 0.16

print('\n── against the mask predicate the chain really uses ──')
hit = 0
for t, tid, d_last in events:
    q = jq[np.argmin(np.abs(jt - t))]
    kin.update(q, np.zeros(7))
    ball = P[np.argmin(np.abs(tt - t))]
    rb = ball - CAM; Lb = np.linalg.norm(rb); ub = rb / Lb
    best = None
    for lab in labels:
        p = kin.control_point_position(lab)
        rp = p - CAM; Lp = np.linalg.norm(rp)
        ang = float(np.arccos(np.clip(np.dot(ub, rp / Lp), -1.0, 1.0)))
        halo = (HALO_EE_PX if 'link7' in lab or 'link8' in lab or 'hand' in lab
                else HALO_BODY_PX) / FX
        if best is None or ang - halo < best[0] - best[1]:
            best = (ang, halo, abs(Lb - Lp), lab)
    ang, halo, ddep, lab = best
    inside = ang <= halo and ddep <= DEPTH_TOL
    hit += inside
    print(f'  t={t:6.2f}  gap {100*d_last:5.1f} cm  angle {1000*ang:5.1f} mrad '
          f'vs halo {1000*halo:4.1f}  Δrange {100*ddep:5.1f} cm  '
          f'-> {"ERASED by the self-mask" if inside else "not the mask"}  ({lab})')
print(f'\nexplained by the self-mask predicate: {hit}/{len(events)}')
