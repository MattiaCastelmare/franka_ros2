"""Construct CBFSafetyFilter with stubbed ROS/pinocchio, to catch __init__ errors."""
import sys, types, traceback
import numpy as np
sys.path.insert(0, 'franka_experiments')

def mod(name, **attrs):
    m = types.ModuleType(name); [setattr(m, k, v) for k, v in attrs.items()]
    sys.modules[name] = m; return m

class Logger:
    def _p(self, *a, **k): print('   [log]', str(a[0])[:120] if a else '')
    info = warn = error = debug = _p
class Clock:
    def now(self): return types.SimpleNamespace(nanoseconds=0)
class FakeNode:
    def __init__(self, name): self._n = name; self._p = {}
    def get_name(self): return self._n
    def get_logger(self): return Logger()
    def get_clock(self): return Clock()
    def declare_parameter(self, n, d): self._p[n] = d
    def get_parameter(self, n): return types.SimpleNamespace(value=self._p[n])
    def create_subscription(self, *a, **k): return None
    def create_publisher(self, *a, **k): return types.SimpleNamespace(publish=lambda m: None)
    def create_timer(self, *a, **k): return None
    def destroy_node(self): pass

import os
mod('ament_index_python')
mod('ament_index_python.packages',
    get_package_share_directory=lambda p: os.path.abspath(p) if os.path.isdir(p) else 'franka_experiments')
mod('rclpy', init=lambda **k: None, shutdown=lambda: None)
mod('rclpy.node', Node=FakeNode)
mod('rclpy.callback_groups', MutuallyExclusiveCallbackGroup=lambda: object())
mod('rclpy.executors', MultiThreadedExecutor=object)
mod('rclpy.qos', QoSProfile=lambda **k: None,
    ReliabilityPolicy=types.SimpleNamespace(BEST_EFFORT=1))
mod('sensor_msgs'); mod('sensor_msgs.msg', JointState=object)
class FMA:
    def __init__(self): self.data = []
mod('std_msgs'); mod('std_msgs.msg', Float64MultiArray=FMA)
mod('franka_msgs'); mod('franka_msgs.msg', MultiLinkDistance=object)

class OSQPProb:
    def setup(self, **k): pass
    def update(self, **k): pass
    def solve(self):
        return types.SimpleNamespace(
            x=np.zeros(13),
            info=types.SimpleNamespace(status_val=1, status='solved', iter=1))
mod('osqp', OSQP=OSQPProb, constant=lambda s: 1)

pin = mod('pinocchio', ReferenceFrame=types.SimpleNamespace(LOCAL_WORLD_ALIGNED=0))
pin.buildModelFromUrdf = lambda p: types.SimpleNamespace(nv=7, nq=7)
pin.neutral = lambda m: np.zeros(9)

import franka_experiments.utils.kinematics as K
class FakeKin:
    def __init__(self, model): self.model = model
    def update(self, *a, **k): pass
    def resolve_frame_id(self, n): return 1
    def point_jacobian(self, f, p):
        J = np.zeros((3, 7)); J[0, 0] = J[1, 1] = J[2, 2] = 1.0
        return J, np.zeros((3, 7))
K.CBFKinematics = FakeKin
K.build_urdf_no_hand = lambda: '/tmp/fake.urdf'
K.build_urdf_with_sc = lambda **k: '/tmp/fake_sc.urdf'

try:
    import franka_experiments.nodes.cbf_safety_filter as N
    N.CBFKinematics = FakeKin
    N.build_urdf_no_hand = K.build_urdf_no_hand
    print('=== import OK, constructing ===')
    node = N.CBFSafetyFilter()
    print('=== CONSTRUCTED OK ===')
except Exception:
    print('=== CONSTRUCTION FAILED ===')
    traceback.print_exc()

# ── drive the two loops with plausible data ─────────────────────────────────
print()
try:
    JS  = N.JointSnap
    OB, OBS = N.Obstacle, N.ObstacleSnap
    q  = np.array([0., -0.6, 0., -1.9, 0., 1.6, 0.])
    qd = np.zeros(7); qd[1] = -0.4
    node._js  = JS(q, qd, 0.0)
    node._nom = N.NomSnap(np.zeros(7), 0.0)
    items = (OB('fr3_link8', 0.25, np.array([.4, 0, .5]), np.array([.4, 0, .25]), 1.0),)
    node._obs = OBS(items, 0.0, 0.0)
    for i in range(5):
        node._update_constraints()
        node._qp_tick()
    print('=== 5 ticks with obstacle OK; n_c =',
          node._con.A.shape[0] if node._con is not None else 0, '===')
    # and the degraded paths
    node._obs = None; node._con = None
    node._qp_tick()
    node._js = JS(q, qd, -99.0)          # stale joint state
    node._qp_tick()
    print('=== degraded paths OK ===')
except Exception:
    print('=== TICK FAILED ===')
    traceback.print_exc()
