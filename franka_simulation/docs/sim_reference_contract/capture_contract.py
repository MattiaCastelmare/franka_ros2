#!/usr/bin/env python3
"""Capture the ROS graph contract of a running stack (provenance of this folder).

Writes, into OUT_DIR:
  graph.json   nodes, per-node pub/sub/service lists, per-topic types and the
               QoS of every endpoint (reliability, durability, history, depth)
  tf.json      every frame seen on /tf and /tf_static (any namespace), its
               parent, whether it is static, and the publishing topic
  params/<node>.yaml   `ros2 param dump` of each node in PARAM_NODES

Usage (inside the container, with the stack already running):
    python3 capture_contract.py OUT_DIR [--wait 5] [--param-node /ns/name ...]
"""
import argparse
import json
import os
import subprocess
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)
from tf2_msgs.msg import TFMessage


def _qos_dict(qos):
    return {
        'reliability': qos.reliability.name if hasattr(qos.reliability, 'name')
        else str(qos.reliability),
        'durability': qos.durability.name if hasattr(qos.durability, 'name')
        else str(qos.durability),
        'history': qos.history.name if hasattr(qos.history, 'name') else str(qos.history),
        'depth': int(qos.depth),
    }


def _graph(node):
    names = sorted(node.get_node_names_and_namespaces())
    out = {'nodes': {}, 'topics': {}}
    for name, ns in names:
        fq = (ns.rstrip('/') + '/' + name) if ns != '/' else '/' + name
        if name.startswith('_contract_capture'):
            continue
        try:
            pubs = node.get_publisher_names_and_types_by_node(name, ns)
            subs = node.get_subscriber_names_and_types_by_node(name, ns)
            srvs = node.get_service_names_and_types_by_node(name, ns)
        except Exception as exc:  # node vanished
            out['nodes'][fq] = {'error': str(exc)}
            continue
        out['nodes'][fq] = {
            'publishers': {t: ty for t, ty in sorted(pubs)},
            'subscribers': {t: ty for t, ty in sorted(subs)},
            'services': sorted(t for t, _ in srvs),
        }
    for topic, types in sorted(node.get_topic_names_and_types()):
        pubs = node.get_publishers_info_by_topic(topic)
        subs = node.get_subscriptions_info_by_topic(topic)

        def _ep(e):
            fq = (e.node_namespace.rstrip('/') + '/' + e.node_name) \
                if e.node_namespace != '/' else '/' + e.node_name
            return {'node': fq, 'qos': _qos_dict(e.qos_profile)}
        out['topics'][topic] = {
            'types': list(types),
            'publishers': sorted((_ep(e) for e in pubs), key=lambda d: d['node']),
            'subscribers': sorted((_ep(e) for e in subs
                                   if not e.node_name.startswith('_contract_capture')),
                                  key=lambda d: d['node']),
        }
    return out


def _tf(node, topics, wait_s):
    frames = {}

    def _cb_factory(topic, static):
        def _cb(msg):
            for t in msg.transforms:
                frames[t.child_frame_id] = {
                    'parent': t.header.frame_id, 'static': static, 'topic': topic,
                    'xyz': [round(t.transform.translation.x, 6),
                            round(t.transform.translation.y, 6),
                            round(t.transform.translation.z, 6)],
                    'quat_xyzw': [round(t.transform.rotation.x, 6),
                                  round(t.transform.rotation.y, 6),
                                  round(t.transform.rotation.z, 6),
                                  round(t.transform.rotation.w, 6)],
                }
        return _cb

    subs = []
    for topic in topics:
        static = topic.endswith('tf_static')
        qos = QoSProfile(depth=100, history=HistoryPolicy.KEEP_LAST,
                         reliability=ReliabilityPolicy.RELIABLE,
                         durability=(DurabilityPolicy.TRANSIENT_LOCAL if static
                                     else DurabilityPolicy.VOLATILE))
        subs.append(node.create_subscription(TFMessage, topic,
                                             _cb_factory(topic, static), qos))
    t_end = time.time() + wait_s
    while time.time() < t_end:
        rclpy.spin_once(node, timeout_sec=0.05)
    return dict(sorted(frames.items()))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('out_dir')
    ap.add_argument('--wait', type=float, default=5.0)
    ap.add_argument('--param-node', action='append', default=[])
    args = ap.parse_args()
    os.makedirs(os.path.join(args.out_dir, 'params'), exist_ok=True)

    rclpy.init()
    node = Node('_contract_capture')
    # Let discovery settle: wait until the node AND topic counts have been
    # unchanged for 3 s (a fixed 2 s wait once caught only 6 of 17 nodes).
    t0 = time.time()
    last, stable_since = None, time.time()
    while time.time() - t0 < 30.0:
        rclpy.spin_once(node, timeout_sec=0.1)
        cur = (len(node.get_node_names_and_namespaces()), len(node.get_topic_names_and_types()))
        if cur != last:
            last, stable_since = cur, time.time()
        elif time.time() - stable_since > 3.0 and time.time() - t0 > 5.0:
            break
    graph = _graph(node)
    tf_topics = [t for t in graph['topics'] if t.endswith('/tf') or t.endswith('/tf_static')]
    tf = _tf(node, tf_topics, args.wait)
    with open(os.path.join(args.out_dir, 'graph.json'), 'w') as f:
        json.dump(graph, f, indent=1, sort_keys=True)
    with open(os.path.join(args.out_dir, 'tf.json'), 'w') as f:
        json.dump({'topics': tf_topics, 'frames': tf}, f, indent=1, sort_keys=True)
    node.destroy_node()
    rclpy.shutdown()

    for fq in args.param_node:
        # --no-daemon discovers from scratch each call; a busy node can be
        # missed on the first try, so retry before recording a failure.
        for _ in range(4):
            res = subprocess.run(['ros2', 'param', 'dump', '--no-daemon', fq],
                                 capture_output=True, text=True, timeout=60)
            if res.returncode == 0 and 'Node not found' not in res.stdout + res.stderr:
                break
            time.sleep(1.0)
        fname = fq.strip('/').replace('/', '__') + '.yaml'
        with open(os.path.join(args.out_dir, 'params', fname), 'w') as f:
            f.write(res.stdout if res.returncode == 0 else f'# FAILED: {res.stderr}\n')
    print(f'captured {len(graph["nodes"])} nodes, {len(graph["topics"])} topics, '
          f'{len(tf)} frames -> {args.out_dir}')


if __name__ == '__main__':
    main()
