#!/usr/bin/env python3
"""ObstacleTrackerNode — republishes MultiLinkDistance with a tracked obstacle
velocity attached to every control point.

WHERE IT SITS
-------------
    real_time_distance ──/cbf/per_link_distances──┐
                                                  ├─> obstacle_tracker_node
    depth image  ─────────────────────────────────┘        │
                                                           │
                          /cbf/per_link_distances_tracked ─┘ ──> cbf_safety_filter

It is a PASS-THROUGH with annotation: every field of every incoming
``LinkDistance`` is copied unchanged and only the four new track fields are
filled. A control point whose nearest obstacle point does not fall inside a
confirmed track keeps the message's documented all-zero defaults, which the
consumer treats as "contribute nothing" — so the worst case for this node is
that it degrades to today's behaviour, never that it changes a distance.

The safety filter is NOT pointed at the new topic by this commit. Repointing it
is a one-line change to ``topics.per_link_distances`` and belongs after the
offline validation in step 6.

WHY IT RUNS ITS OWN DistanceEngine
----------------------------------
The clusters must be built from EXACTLY the pixels the distance pass selected —
same exclusion mask, same depth-range filter, same ROI stride — or the tracker
would report velocities for obstacles the barrier never saw. Rather than
re-deriving that selection (and drifting from it the first time either side is
touched), this node instantiates the SAME ``MaskBuilder`` and ``DistanceEngine``
from the SAME config and calls the same ``compute()``, then reads the
``ObstacleCloud`` the engine exports. Identical semantics by construction.

The cost is one duplicated mask build plus one distance pass per frame, a few
milliseconds at 30 Hz on a pipeline that is not real-time critical (the QP runs
off the SEPARATE, already-published distance message; nothing waits on this
node). That is a deliberate trade of CPU for the guarantee that the two pixel
sets cannot diverge. Once step 6 validates the estimate, the natural end state
is to fold the pipeline into ``real_time_distance``, which already has the
cloud in hand — at which point the duplication disappears.

FAIL-SOFT, ALWAYS
-----------------
Any failure inside the tracking path — TF missing, cluster blow-up, filter
exception — must still let the annotated message go out with zeroed track
fields. A safety filter downstream that stops receiving distances brakes; one
that receives distances with no track simply falls back to the residual
estimator it uses today. The exception handlers here choose the second every
time.
"""
from __future__ import annotations

import os
import queue
import threading
from typing import Optional

import numpy as np
import rclpy
import trimesh
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from franka_msgs.msg import MultiLinkDistance
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener

from franka_experiments.utils.distance_engine import DistanceEngine
from franka_experiments.utils.distance_utils import (
    compute_roi,
    define_control_points,
    load_extrinsics,
    load_robot_config,
)
from franka_experiments.utils.mask_builder import MaskBuilder
from franka_experiments.utils.obstacle_track_pipeline import ObstacleTrackPipeline
from franka_experiments.utils.params import declare_bool, declare_float, declare_int, declare_str
from franka_experiments.utils.tf_manager import TFManager


def annotate_links(msg: MultiLinkDistance, pipeline: ObstacleTrackPipeline) -> int:
    """Fill the track fields of every entry of ``msg`` IN PLACE.

    Kept as a free function, and taking only the message and the pipeline, so
    the annotation rule is unit-testable without a node, a camera or a clock.

    Returns:
        How many entries were matched to a confirmed track.
    """
    n = 0
    for ld in msg.links:
        if not ld.valid:
            # An invalid entry has no meaningful closest_point_human — annotating
            # it would attach a velocity to a point that was never measured.
            continue
        p_base = np.array([ld.closest_point_human.x,
                           ld.closest_point_human.y,
                           ld.closest_point_human.z])
        tid, seen, v, P = pipeline.velocity_for_point(p_base)
        ld.track_id = int(tid)
        ld.frames_seen = int(seen)
        ld.obstacle_velocity.x = float(v[0])
        ld.obstacle_velocity.y = float(v[1])
        ld.obstacle_velocity.z = float(v[2])
        ld.velocity_covariance = np.asarray(P, dtype=np.float64).ravel()
        if tid:
            n += 1
    return n


class ObstacleTrackerNode(Node):

    def __init__(self):
        super().__init__('obstacle_tracker')

        robot_config_path      = declare_str(self, 'robot_config_path', '')
        camera_extrinsics_path = declare_str(self, 'camera_extrinsics_path', '')

        self.config       = load_robot_config(robot_config_path)
        self.robot_cfg    = self.config['robot']
        self.distance_cfg = dict(self.config['distance'])
        self.mask_cfg     = self.config['mask']
        self.mesh_cfg     = self.config['meshes']

        # THE flag that makes the engine hand back its pixel selection. Set here
        # rather than in the YAML so real_time_distance's own engine keeps the
        # export off and pays nothing for a feature it does not use.
        self.distance_cfg['export_obstacle_cloud'] = True

        self.ee_link = self.robot_cfg.get('ee_link', 'fr3_link8')
        self.R_base, self.t_base = load_extrinsics(camera_extrinsics_path)
        self.R_base_f32 = self.R_base.astype(np.float32)
        self.t_base_f32 = self.t_base.astype(np.float32)

        # ── Tracker parameters ───────────────────────────────────────────
        # Declared on the node (not read from the CBF config) because they tune
        # PERCEPTION, and the safety filter must not have to be restarted to
        # retune a camera-side filter. Every default is the module default —
        # see obstacle_tracker.py for the sweep behind q_jerk.
        self.pipeline = ObstacleTrackPipeline(
            voxel_m=declare_float(self, 'cluster_voxel_m', 0.02,
                                  minimum=0.005, maximum=0.2),
            min_cluster_points=declare_int(self, 'cluster_min_points', 10,
                                           minimum=1, maximum=10000),
            max_clusters=declare_int(self, 'max_clusters', 16,
                                     minimum=1, maximum=128),
            contains_tol=declare_float(self, 'cluster_contains_tol_m', 0.05,
                                       minimum=0.0, maximum=1.0),
            q_jerk=declare_float(self, 'track_q_jerk', 2.0,
                                 positive=True, maximum=1000.0),
            sigma_meas=declare_float(self, 'track_sigma_meas_m', 0.01,
                                     positive=True, maximum=1.0),
            gate_mahalanobis=declare_float(self, 'track_gate_mahalanobis', 3.0,
                                           positive=True, maximum=20.0),
            gate_max_m=declare_float(self, 'track_gate_max_m', 0.5,
                                     positive=True, maximum=5.0),
            confirm_hits=declare_int(self, 'track_confirm_hits', 3,
                                     minimum=1, maximum=20),
            confirm_window=declare_int(self, 'track_confirm_window', 5,
                                       minimum=1, maximum=50),
            max_missed=declare_int(self, 'track_max_missed', 5,
                                   minimum=0, maximum=50),
            max_tracks=declare_int(self, 'track_max_tracks', 12,
                                   minimum=1, maximum=64),
        )
        self._log_period_s = declare_float(self, 'log_period_s', 5.0,
                                           minimum=0.0, maximum=600.0)
        self._passthrough_only = declare_bool(self, 'passthrough_only', False)

        # ── Camera / TF / sub-systems (mirrors real_time_distance) ────────
        self.bridge = CvBridge()
        self.cx_f32 = self.cy_f32 = None
        self.fx_inv_f32 = self.fy_inv_f32 = None

        self.tf_buffer   = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_mgr = TFManager(
            tf_buffer=self.tf_buffer,
            base_frame=self.robot_cfg['base_frame'],
            critical_links=self.robot_cfg.get('critical_links', [self.ee_link]),
            cache_max_age_s=float(self.distance_cfg.get('tf_cache_max_age_s', 0.5)),
            logger=self.get_logger(),
        )

        mesh_base_dir = get_package_share_directory(
            self.mesh_cfg.get('package', 'franka_description'))
        sample_pts = int(self.mesh_cfg.get('sample_points_per_link', 300))
        link_mesh_samples = {
            name: trimesh.load(os.path.join(mesh_base_dir, rel),
                               force='mesh').sample(sample_pts)
            for name, rel in self.mesh_cfg['files'].items()
        }
        self.mask_builder = MaskBuilder(
            link_mesh_samples=link_mesh_samples, R_base=self.R_base,
            t_base=self.t_base, ee_link=self.ee_link, mask_cfg=self.mask_cfg,
            logger=self.get_logger())
        self.distance_engine = DistanceEngine(
            distance_cfg=self.distance_cfg, logger=self.get_logger())

        self._last_depth_shape: Optional[tuple] = None
        self.roi_bounds: Optional[tuple] = None
        # The pipeline's state is read by the distance callback and written by
        # the depth thread. Both run under the GIL, but `update()` is not atomic
        # and an annotation taken halfway through it would mix two frames'
        # clusters. One lock, held only around the two short critical sections.
        self._lock = threading.Lock()
        self._frame_queue: queue.Queue = queue.Queue(maxsize=1)
        self._n_dropped = 0
        self._n_annotated = 0
        self._n_msgs = 0

        # ── Topics ───────────────────────────────────────────────────────
        topics = self.config.get('topics', {})
        depth_topic = topics.get('depth_image',
                                 '/camera/camera/depth/image_rect_raw')
        info_topic = topics.get('depth_camera_info',
                                '/camera/camera/depth/camera_info')
        in_topic = topics.get('per_link_distances', '/cbf/per_link_distances')
        out_topic = topics.get('per_link_distances_tracked',
                               in_topic + '_tracked')

        be_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Image, depth_topic, self._on_depth, 10)
        self.create_subscription(CameraInfo, info_topic, self._on_camera_info, 10)
        self.create_subscription(MultiLinkDistance, in_topic,
                                 self._on_distances, be_qos)
        self._pub = self.create_publisher(MultiLinkDistance, out_topic, be_qos)

        self._thread = threading.Thread(target=self._cluster_loop,
                                        name='obstacle_tracker', daemon=True)
        self._thread.start()
        if self._log_period_s > 0.0:
            self.create_timer(self._log_period_s, self._log_status)

        self.get_logger().info(
            f'obstacle_tracker ready\n'
            f'  in : {in_topic}\n'
            f'  out: {out_topic}\n'
            f'  depth: {depth_topic}\n'
            f'  passthrough_only={self._passthrough_only}')

    # ── Camera ──────────────────────────────────────────────────────────────

    def _on_camera_info(self, msg: CameraInfo) -> None:
        K = np.array(msg.k, dtype=np.float64).reshape(3, 3)
        self.cx_f32 = np.float32(K[0, 2])
        self.cy_f32 = np.float32(K[1, 2])
        self.fx_inv_f32 = np.float32(1.0 / K[0, 0])
        self.fy_inv_f32 = np.float32(1.0 / K[1, 1])
        self.mask_builder.set_intrinsics(K)

    def _on_depth(self, msg: Image) -> None:
        try:
            depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception as exc:
            self.get_logger().error(f'depth conversion failed: {exc}',
                                    throttle_duration_sec=2.0)
            return
        # maxsize=1 + drop-oldest: clustering must never queue up behind a slow
        # frame. A stale cloud is worse than a skipped one — the tracks would be
        # advanced with a dt that no longer matches the data.
        try:
            self._frame_queue.put_nowait((depth, msg.header.stamp))
        except queue.Full:
            self._n_dropped += 1
            try:
                self._frame_queue.get_nowait()
                self._frame_queue.put_nowait((depth, msg.header.stamp))
            except (queue.Empty, queue.Full):
                pass

    # ── Clustering / tracking thread ────────────────────────────────────────

    def _cluster_loop(self) -> None:
        while rclpy.ok():
            try:
                frame = self._frame_queue.get(timeout=0.1)
            except queue.Empty:
                continue
            try:
                self._process(frame)
            except Exception as exc:
                # Fail soft: the annotated message still goes out with zeroed
                # track fields, which is exactly today's behaviour.
                self.get_logger().error(f'tracking skipped this frame: {exc}',
                                        throttle_duration_sec=2.0)

    def _process(self, frame) -> None:
        depth, stamp = frame
        if self.cx_f32 is None or self._passthrough_only:
            return

        if self._last_depth_shape != depth.shape:
            self._last_depth_shape = depth.shape
            self.mask_builder.invalidate()
            self.distance_engine.invalidate_grid_cache()
            self.distance_engine.reset_lpf()
            self.roi_bounds = None
            with self._lock:
                self.pipeline.reset()

        H, W = depth.shape
        step = int(self.distance_cfg['pixel_step'])
        margin = int(self.distance_cfg['image_margin_px'])

        transforms = self.tf_mgr.lookup_all(self.robot_cfg['segment_links'], stamp)
        control_points = define_control_points(
            transforms, self.robot_cfg, self.distance_cfg)
        if not control_points:
            return

        self.mask_builder.rebuild(transforms, depth.shape)
        self.roi_bounds = compute_roi(
            self.mask_builder.search_exclusion_mask, H, W, margin,
            int(self.distance_cfg['roi_pad_px'])) or (margin, margin,
                                                      W - margin, H - margin)
        x0, y0, x1, y1 = self.roi_bounds
        t_cap = stamp.sec + stamp.nanosec * 1e-9

        # Same call, same arguments, same config as real_time_distance — the
        # cloud it exports is the identical pixel selection. The returned
        # distances are discarded on purpose: this node does not publish
        # distances, it annotates the ones real_time_distance already published.
        self.distance_engine.compute(
            depth=depth, cx_f32=self.cx_f32, cy_f32=self.cy_f32,
            fx_inv_f32=self.fx_inv_f32, fy_inv_f32=self.fy_inv_f32,
            R_base_f32=self.R_base_f32, t_base_f32=self.t_base_f32,
            control_points=control_points,
            x=np.array([x0, x1]), y=np.array([y0, y1]), step=step,
            search_exclusion_mask=self.mask_builder.search_exclusion_mask,
            transforms=transforms,
            ee_source_mask=self.mask_builder.ee_source_mask,
            dilation_margins_px=self.mask_builder.dilation_margins_px,
            frame_stamp=t_cap,
        )
        cloud = self.distance_engine.last_obstacle_cloud
        with self._lock:
            self.pipeline.update(cloud, self.R_base, self.t_base, stamp=t_cap)

    # ── Distance passthrough ────────────────────────────────────────────────

    def _on_distances(self, msg: MultiLinkDistance) -> None:
        self._n_msgs += 1
        try:
            if not self._passthrough_only:
                with self._lock:
                    self._n_annotated += annotate_links(msg, self.pipeline)
        except Exception as exc:
            self.get_logger().error(f'annotation skipped: {exc}',
                                    throttle_duration_sec=2.0)
        # Published even when annotation failed: a consumer that stops receiving
        # distances brakes, one that receives them without a track falls back.
        self._pub.publish(msg)

    def _log_status(self) -> None:
        with self._lock:
            desc = self.pipeline.describe()
        self.get_logger().info(
            f'OBSTRK {desc} msgs={self._n_msgs} annotated={self._n_annotated} '
            f'dropped_frames={self._n_dropped}')


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleTrackerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
