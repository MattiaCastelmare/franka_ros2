"""Capture a running ROS graph and diff it against the Phase 0 reference contract.

The reference (``franka_simulation/docs/sim_reference_contract/fake_hw/``) is
pipeline A on fake hardware. The sim must reproduce it on the SAFETY PATH:
same nodes, same topics with the same types, the same endpoints with the same
reliability and durability, and the same TF tree. It may differ elsewhere:
rates (fake hardware runs at 100 Hz, the robot and Gazebo at 1 kHz), extra sim
nodes (bridges, truth, reflex monitor, recorders), extra frames (hand, camera).

``capture(node, ...)`` needs rclpy; ``diff(ref, sim)`` is pure.
"""
from __future__ import annotations

import json
import os
import time

#: Pipeline-A nodes whose endpoints form the contract (plan §1, H7).
SAFETY_NODES = (
    '/NS_1/controller_manager', '/NS_1/joint_state_broadcaster', '/NS_1/rt_torque_controller',
    '/NS_1/robot_state_publisher', '/NS_1/joint_state_publisher',
    '/NS_1/pentagon_qddot_commander', '/cbf_safety_filter', '/qddot_to_torque',
    '/real_time_distance', '/experiment_logger', '/world_to_robot_base_tf',
)
SAFETY_TOPICS = (
    '/NS_1/franka/joint_states', '/NS_1/joint_states', '/NS_1/qddot_nom', '/NS_1/qddot_safe',
    '/NS_1/torque_cmd', '/NS_1/cbf_status', '/cbf/per_link_distances', '/NS_1/iso_safety',
    '/NS_1/torque_saturation', '/NS_1/ee_actual', '/NS_1/ee_desired', '/tf', '/tf_static',
    '/camera/camera/depth/image_rect_raw', '/camera/camera/depth/camera_info',
)


def _fq(name: str, ns: str) -> str:
    return (ns.rstrip('/') + '/' + name) if ns != '/' else '/' + name


def _qos(q) -> dict:
    return {'reliability': q.reliability.name, 'durability': q.durability.name,
            'history': q.history.name, 'depth': int(q.depth)}


def capture_graph(node, settle_s: float = 3.0, max_s: float = 30.0) -> dict:
    """Same schema as the Phase 0 graph.json. Waits until discovery is stable."""
    import rclpy
    t0 = time.time()
    last, since = None, time.time()
    while time.time() - t0 < max_s:
        rclpy.spin_once(node, timeout_sec=0.1)
        cur = (len(node.get_node_names_and_namespaces()), len(node.get_topic_names_and_types()))
        if cur != last:
            last, since = cur, time.time()
        elif time.time() - since > settle_s and time.time() - t0 > 2 * settle_s:
            break
    me = node.get_name()
    out = {'nodes': {}, 'topics': {}}
    for name, ns in sorted(node.get_node_names_and_namespaces()):
        if name == me:
            continue
        try:
            out['nodes'][_fq(name, ns)] = {
                'publishers': dict(sorted(node.get_publisher_names_and_types_by_node(name, ns))),
                'subscribers': dict(sorted(node.get_subscriber_names_and_types_by_node(name, ns))),
                'services': sorted(t for t, _ in node.get_service_names_and_types_by_node(name, ns)),
            }
        except Exception as exc:                            # noqa: BLE001
            out['nodes'][_fq(name, ns)] = {'error': str(exc)}
    for topic, types in sorted(node.get_topic_names_and_types()):
        def ep(e):
            return {'node': _fq(e.node_name, e.node_namespace), 'qos': _qos(e.qos_profile)}
        out['topics'][topic] = {
            'types': list(types),
            'publishers': sorted((ep(e) for e in node.get_publishers_info_by_topic(topic)
                                  if e.node_name != me), key=lambda d: d['node']),
            'subscribers': sorted((ep(e) for e in node.get_subscriptions_info_by_topic(topic)
                                   if e.node_name != me), key=lambda d: d['node']),
        }
    return out


def capture_tf(node, wait_s: float = 5.0) -> dict:
    import rclpy
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from tf2_msgs.msg import TFMessage
    frames = {}

    def cb_for(topic, static):
        def cb(msg):
            for t in msg.transforms:
                frames[t.child_frame_id] = {
                    'parent': t.header.frame_id, 'static': static, 'topic': topic,
                    'xyz': [round(t.transform.translation.x, 6), round(t.transform.translation.y, 6),
                            round(t.transform.translation.z, 6)]}
        return cb
    subs = []
    for topic, static in (('/tf', False), ('/tf_static', True)):
        qos = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE,
                         durability=(DurabilityPolicy.TRANSIENT_LOCAL if static
                                     else DurabilityPolicy.VOLATILE))
        subs.append(node.create_subscription(TFMessage, topic, cb_for(topic, static), qos))
    t_end = time.time() + wait_s
    while time.time() < t_end:
        rclpy.spin_once(node, timeout_sec=0.05)
    for s in subs:
        node.destroy_subscription(s)
    return {'topics': ['/tf', '/tf_static'], 'frames': dict(sorted(frames.items()))}


def reference_dir() -> str:
    here = os.path.dirname(os.path.realpath(__file__))
    for cand in (os.path.join(here, '..', '..', '..', 'franka_simulation', 'docs',
                              'sim_reference_contract', 'fake_hw'),
                 '/ros2_ws/src/franka_simulation/docs/sim_reference_contract/fake_hw'):
        if os.path.isfile(os.path.join(cand, 'graph.json')):
            return os.path.normpath(cand)
    raise FileNotFoundError('Phase 0 reference contract not found')


def load_reference(d: str | None = None) -> tuple[dict, dict]:
    d = d or reference_dir()
    with open(os.path.join(d, 'graph.json')) as f:
        g = json.load(f)
    with open(os.path.join(d, 'tf.json')) as f:
        tf = json.load(f)
    return g, tf


def _endpoints(eps, nodes):
    return sorted((e['node'], e['qos']['reliability'], e['qos']['durability'])
                  for e in eps if e['node'] in nodes)


def diff(ref_graph: dict, sim_graph: dict, ref_tf: dict | None = None,
         sim_tf: dict | None = None, nodes=SAFETY_NODES, topics=SAFETY_TOPICS) -> list[str]:
    """Contract violations on the safety path; [] when the sim reproduces it."""
    problems = []
    ref_nodes = [n for n in nodes if n in ref_graph['nodes']]
    for n in ref_nodes:
        if n not in sim_graph['nodes']:
            problems.append(f'node missing: {n}')
    for t in topics:
        r = ref_graph['topics'].get(t)
        s = sim_graph['topics'].get(t)
        if r is None:
            continue
        if s is None:
            problems.append(f'topic missing: {t}')
            continue
        if r['types'] != s['types']:
            problems.append(f'{t}: type {s["types"]} != {r["types"]}')
        for side in ('publishers', 'subscribers'):
            a, b = _endpoints(r[side], ref_nodes), _endpoints(s[side], ref_nodes)
            if a != b:
                problems.append(f'{t} {side}: sim {b} != ref {a}')
    if ref_tf is not None and sim_tf is not None:
        for frame, rf in ref_tf['frames'].items():
            sf = sim_tf['frames'].get(frame)
            if sf is None:
                problems.append(f'TF frame missing: {frame}')
                continue
            if sf['parent'] != rf['parent'] or sf['static'] != rf['static']:
                problems.append(f'TF {frame}: parent/static {sf["parent"]}/{sf["static"]} '
                                f'!= {rf["parent"]}/{rf["static"]}')
            elif rf['static'] and max(abs(x - y) for x, y in zip(sf['xyz'], rf['xyz'])) > 1e-6:
                problems.append(f'TF {frame}: static offset {sf["xyz"]} != {rf["xyz"]}')
    return problems
