#!/usr/bin/env python3
"""Experimental RGB-D object evidence using ACTIVE landmarks 0, 5, 9, 17.

No grasp generation or control. Palm-seeded colour rejection is a baseline,
not semantic hand segmentation: similar-coloured objects can be missed.
"""
from collections import deque
from pathlib import Path

import cv2
import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge, CvBridgeError
from message_filters import Subscriber, TimeSynchronizer
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image

from franka_msgs.msg import HandObjectState, HandState, HandTrackingRaw
from franka_experiments.utils.config import load_config_file, load_extrinsics
from franka_experiments.utils.node_runtime import teardown


def object_candidate(bgr, depth_m, points, k, radius=0.14):
    """Return (observable, camera centroid or None, score), all lengths in m.

    Reject truncated/ambiguous clusters. Four RGB-D palm points seed an
    adaptive colour mask; no fixed skin-colour threshold or object class.
    """
    if (bgr.shape[:2] != depth_m.shape or points.shape != (4, 3)
            or not np.isfinite(points).all() or np.any(points[:, 2] <= .1)):
        return False, None, 0.0
    fx, fy, cx, cy = k
    palm = points[1:].mean(axis=0)
    uv = points[:, :2] / points[:, 2, None] * [fx, fy] + [cx, cy]
    center = palm[:2] / palm[2] * [fx, fy] + [cx, cy]
    margin = np.array([fx, fy]) * radius / max(.1, palm[2] - radius)
    lo = np.floor(center - margin).astype(int)
    hi = np.ceil(center + margin).astype(int)
    h, w = depth_m.shape
    if np.any(lo < 0) or hi[0] >= w or hi[1] >= h:
        return False, None, 0.0
    x0, y0 = lo
    x1, y1 = hi
    z = depth_m[y0:y1+1, x0:x1+1]
    yy, xx = np.mgrid[y0:y1+1, x0:x1+1]
    xyz = np.stack(((xx-cx)*z/fx, (yy-cy)*z/fy, z), axis=-1)
    local = np.isfinite(z) & (z > .1) & (np.linalg.norm(xyz-palm, axis=-1) < radius)
    hull = cv2.convexHull(np.rint(uv-lo).astype(np.int32))
    core = np.zeros(z.shape, np.uint8)
    cv2.fillConvexPoly(core, hull, 1)
    seed = cv2.erode(core, np.ones((3, 3), np.uint8)).astype(bool) & local
    if seed.sum() < 8 or local.sum() < 20:
        return False, None, 0.0
    lab = cv2.cvtColor(bgr[y0:y1+1, x0:x1+1], cv2.COLOR_BGR2LAB).astype(float)
    # Reduce illumination sensitivity; chroma is sampled on this hand/frame.
    lab *= [.25, 1., 1.]
    color = np.median(lab[seed], axis=0)
    spread = np.percentile(np.linalg.norm(lab[seed]-color, axis=1), 90)
    if spread > 25.:
        return False, None, 0.0
    hand = (np.linalg.norm(lab-color, axis=-1) <= max(18., 2.*spread)) | core.astype(bool)
    # Aligned depth can smear foreground depth beyond RGB silhouettes. Exclude
    # a metric border instead of promoting these edge fragments to objects.
    border = max(1, int(np.ceil(max(fx, fy)*.015/palm[2])))
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2*border+1, 2*border+1))
    hand = cv2.dilate(hand.astype(np.uint8), kernel).astype(bool)
    # Exclude the forearm side of the wrist using the observed palm axis.
    axis = palm-points[0]
    norm = np.linalg.norm(axis)
    if norm < .015:
        return False, None, 0.0
    forward = axis/norm
    lateral = points[3]-points[1]
    width = np.linalg.norm(lateral)
    normal = np.cross(forward, lateral)
    normal_length = np.linalg.norm(normal)
    if width < .02 or normal_length < .01:
        return False, None, 0.0
    normal /= normal_length
    lateral = np.cross(normal, forward)
    offset = xyz-points[0]
    along = offset @ forward
    # Four points cannot locate fingertips: exclude a conservative hand volume.
    # Objects entirely inside this volume are deliberately unresolvable here.
    envelope = ((along >= -.01) & (along <= 2.6*norm)
                & (np.abs(offset @ lateral) <= width/2.+.025)
                & (np.abs(offset @ normal) <= .025))
    candidate = local & ~hand & ~envelope & (along > .02)
    # Break image-space connections across depth discontinuities.
    edges = np.zeros(z.shape, bool)
    edges[1:] |= np.abs(z[1:]-z[:-1]) > .02
    edges[:, 1:] |= np.abs(z[:, 1:]-z[:, :-1]) > .02
    candidate &= ~edges
    _, labels = cv2.connectedComponents(candidate.astype(np.uint8), connectivity=4)
    clusters = []
    for label, n in enumerate(np.bincount(labels.ravel())):
        if label == 0 or n < 8:
            continue
        mask = labels == label
        cloud = xyz[mask]
        centroid = np.median(cloud, axis=0)
        extent = np.ptp(cloud, axis=0)
        area = n * float(np.median(cloud[:, 2]))**2 / (fx*fy)
        # Clusters cut by the spherical ROI may be table/body/robot surfaces.
        if (area < .00015 or np.max(extent) < .02 or np.max(extent) > .16
                or np.max(np.linalg.norm(cloud-palm, axis=1)) > .95*radius
                or np.linalg.norm(centroid-palm) > .12
                or np.min(np.linalg.norm(cloud[:, None]-points[None], axis=2)) > .045):
            continue
        clusters.append((centroid, float(min(1., area/.0006))))
    if len(clusters) > 1:
        return False, None, 0.0
    return (True, *clusters[0]) if clusters else (True, None, 0.0)


class ObjectHistory:
    """Confirm fresh observations; never carry evidence across hand switches."""
    def __init__(self, confirm_s=.3, release_s=.15, max_gap_s=.2):
        self.confirm_s, self.release_s, self.max_gap_s = confirm_s, release_s, max_gap_s
        self.reset()

    def reset(self):
        self.side = 0
        self.last_t = self.start = self.seen = None
        self.offset = self.centroid = None
        self.hits = 0
        self.present = False
        self.confidence = 0.0

    def update(self, t, side, valid, centroid, palm, score):
        if (side != self.side or (self.last_t is not None
                and (t <= self.last_t or t-self.last_t > self.max_gap_s))):
            self.reset()
        if not valid or side not in (1, 2):
            self.reset()
            return
        self.side, self.last_t = side, t
        if centroid is None:
            self.start, self.hits = None, 0
            if self.seen is None or t-self.seen >= self.release_s:
                self.present = False
            return
        offset = centroid-palm
        if self.offset is None or np.linalg.norm(offset-self.offset) > .04:
            self.start, self.hits, self.present = t, 0, False
            self.offset = offset.copy()
        if self.start is None:
            self.start = t
        self.hits += 1
        self.seen = t
        self.centroid, self.confidence = centroid.copy(), score
        self.present |= self.hits >= 3 and t-self.start >= self.confirm_s


class Grasp(Node):
    def __init__(self):
        super().__init__('grasp')
        config = Path(get_package_share_directory('franka_experiments')) / 'config/camera_extrinsics.yaml'
        self.rotation, self.translation = load_extrinsics(str(config))
        self.base_frame = load_config_file(str(config))['parent_frame']
        self.radius = float(self.declare_parameter('roi_radius_m', .14).value)
        self.timeout = float(self.declare_parameter('input_timeout_s', .25).value)
        confirm = float(self.declare_parameter('confirm_s', .3).value)
        release = float(self.declare_parameter('release_s', .15).value)
        if not (np.isfinite([self.radius, self.timeout, confirm, release]).all()
                and .05 <= self.radius <= .25 and self.timeout > 0 and confirm > 0 and release > 0):
            raise ValueError('Invalid grasp ROI or timing parameters')
        self.history = ObjectHistory(confirm, release)
        self.bridge, self.k = CvBridge(), None
        self.rgb, self.depth = deque(maxlen=12), deque(maxlen=12)
        self.last_input = None
        self.publisher = self.create_publisher(HandObjectState, '/handover/hand_object', 1)
        self.create_subscription(CameraInfo, '/camera/camera/aligned_depth_to_color/camera_info', self.camera_info, qos_profile_sensor_data)
        self.create_subscription(Image, '/camera/camera/color/image_raw', self.rgb.append, qos_profile_sensor_data)
        self.create_subscription(Image, '/camera/camera/aligned_depth_to_color/image_raw', self.depth.append, qos_profile_sensor_data)
        self.raw_sub = Subscriber(self, HandTrackingRaw, '/handover/hand_tracking_raw')
        self.state_sub = Subscriber(self, HandState, '/handover/hand_state')
        self.sync = TimeSynchronizer([self.raw_sub, self.state_sub], 8)
        self.sync.registerCallback(self.callback)
        self.create_timer(.1, self.watchdog)

    @staticmethod
    def stamp(msg):
        return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

    def camera_info(self, msg):
        k = np.array([msg.k[0], msg.k[4], msg.k[2], msg.k[5]])
        self.k = k if np.isfinite(k).all() and min(k[:2]) > 0 else None

    def callback(self, raw, state):
        t = self.stamp(state)
        self.last_input = state
        age = self.get_clock().now().nanoseconds*1e-9-t
        valid = bool(0 <= age <= self.timeout and state.position_valid and state.position_fresh
                     and state.position_source == HandState.POSITION_SOURCE_MEASURED
                     and state.physical_hand in (1, 2) and raw.handedness == state.physical_hand
                     and state.header.frame_id == raw.header.frame_id == self.base_frame
                     and raw.tracking_state == HandTrackingRaw.TRACKING_FULL
                     and list(raw.landmark_ids) == [0, 5, 9, 17]
                     and all(raw.valid) and all(v == raw.DIRECT for v in raw.measurement_type))
        centroid, score = None, 0.0
        palm = np.array([state.palm_position.x, state.palm_position.y, state.palm_position.z])
        rgb = min(self.rgb, key=lambda m: abs(self.stamp(m)-t), default=None)
        depth = min(self.depth, key=lambda m: abs(self.stamp(m)-t), default=None)
        valid &= bool(self.k is not None and np.isfinite(palm).all() and rgb is not None and depth is not None
                      and abs(self.stamp(rgb)-t) < .001 and abs(self.stamp(depth)-t) <= .02)
        if valid:
            try:
                bgr = self.bridge.imgmsg_to_cv2(rgb, 'bgr8')
                z = self.bridge.imgmsg_to_cv2(depth, 'passthrough').astype(np.float32)
                if depth.encoding == '16UC1':
                    z *= .001
                elif depth.encoding != '32FC1':
                    raise ValueError('Unsupported depth encoding')
                points = np.array([[p.x, p.y, p.z] for p in raw.positions])
                points = (points-self.translation) @ self.rotation
                valid, camera_centroid, score = object_candidate(bgr, z, points, self.k, self.radius)
                if camera_centroid is not None:
                    centroid = self.rotation @ camera_centroid + self.translation
            except (ValueError, cv2.error, CvBridgeError) as error:
                self.get_logger().warning(str(error), throttle_duration_sec=2.)
                valid = False
        self.history.update(t, state.physical_hand, valid, centroid, palm, score)
        self.publish(state, t, valid)

    def publish(self, state, t, valid):
        out = HandObjectState()
        out.header, out.physical_hand = state.header, state.physical_hand
        out.valid = bool(valid)
        out.object_present = bool(valid and self.history.present)
        out.object_age = float('inf') if self.history.seen is None else max(0., t-self.history.seen)
        out.object_confidence = float(self.history.confidence * max(0., 1.-out.object_age/self.history.release_s)) if out.object_present else 0.
        point = self.history.centroid if out.object_present else [float('nan')]*3
        out.object_centroid_3d.x, out.object_centroid_3d.y, out.object_centroid_3d.z = map(float, point)
        self.publisher.publish(out)

    def watchdog(self):
        if self.last_input is not None:
            now = self.get_clock().now().nanoseconds*1e-9
            if not 0 <= now-self.stamp(self.last_input) <= self.timeout:
                self.history.reset()
                self.publish(self.last_input, now, False)
                self.last_input = None


def main(args=None):
    rclpy.init(args=args)
    node = Grasp()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        teardown(node)
