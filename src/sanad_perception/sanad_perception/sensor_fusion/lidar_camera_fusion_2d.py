"""
depth_fusion_2d.py
==================
Fused 2D LaserScan producer for the sanad autonomous wheelchair project.

Pipeline
--------
  1.  ApproximateTimeSynchronizer aligns RGBD PointCloud2 + raw LaserScan.
  2.  On the first synchronized pair that faces a flat wall, the
      RANSAC-based depth-bias calibration runs and computes Δz.
  3.  Every subsequent frame applies Δz to the RGBD Z-axis before
      projecting the cloud to 2D.
  4.  The LiDAR scan is re-binned onto the same angular grid.
  5.  A range-adaptive weighted fusion merges both bin arrays.
  6.  A valid LaserScan is published on /fused_scan.

Calibration functions are module-level (outside the ROS node class) so
they can be unit-tested independently.

References
----------
  Delhibabu et al., "2D grid map creation based on RGBD-camera and
  LiDAR data", Scientific Reports 16:10591 (2026).
"""

from __future__ import annotations

import numpy as np
import rclpy
import rclpy.duration
from message_filters import ApproximateTimeSynchronizer, Subscriber
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import LaserScan, PointCloud2
from sensor_msgs_py import point_cloud2
from tf2_ros import Buffer, TransformListener

class DepthFusionNode2D(Node):
    """
    Fused 2D LaserScan node for the sanad autonomous wheelchair.

    Subscribes
    ----------
    /realsense/points  (sensor_msgs/PointCloud2)
    /scan              (sensor_msgs/LaserScan)

    Publishes
    ---------
    /fused_scan        (sensor_msgs/LaserScan)
    """

    def __init__(self):
        super().__init__("depth_fusion_2d")

        # ── Output grid parameters ───────────────────────────────────────────
        self.declare_parameter("angle_min",       -3.14159)
        self.declare_parameter("angle_max",        3.14159)
        self.declare_parameter("angle_increment",  0.00581)   # ~0.333 deg
        self.declare_parameter("range_min",        0.1)
        self.declare_parameter("range_max",       12.0)

        # ── Height filter ────────────────────────────────────────────────────
        self.declare_parameter("min_height",  0.1)   # floor clearance (m)
        self.declare_parameter("max_height",  2.0)   # ceiling / max obstacle height

        # ── Fusion parameters ────────────────────────────────────────────────
        self.declare_parameter("sigma_rgbd",  1.5)   # RGBD range-decay constant
        self.declare_parameter("alpha",       1.0)   # LiDAR weight (constant)

        # ── Calibration parameters ───────────────────────────────────────────
        self.declare_parameter("auto_calibrate",          True)
        self.declare_parameter("calibration_attempts",    5)    # average over N attempts
        self.declare_parameter("frontal_half_angle_deg", 20.0)  # LiDAR frontal cone
        self.declare_parameter("target_frame",       "base_footprint")
        self.declare_parameter("rgbd_topic",         "/realsense/points")
        self.declare_parameter("lidar_topic",        "/scan")

        # ── Read params ──────────────────────────────────────────────────────
        self.angle_min       = self.get_parameter("angle_min").value
        self.angle_max       = self.get_parameter("angle_max").value
        self.angle_increment = self.get_parameter("angle_increment").value
        self.range_min       = self.get_parameter("range_min").value
        self.range_max       = self.get_parameter("range_max").value
        self.min_height      = self.get_parameter("min_height").value
        self.max_height      = self.get_parameter("max_height").value
        self.sigma_rgbd      = self.get_parameter("sigma_rgbd").value
        self.alpha           = self.get_parameter("alpha").value
        self.auto_calibrate  = self.get_parameter("auto_calibrate").value
        self.cal_attempts    = self.get_parameter("calibration_attempts").value
        self.frontal_angle   = self.get_parameter("frontal_half_angle_deg").value
        self.target_frame    = self.get_parameter("target_frame").value
        rgbd_topic           = self.get_parameter("rgbd_topic").value
        lidar_topic          = self.get_parameter("lidar_topic").value

        self.num_bins = int(
            (self.angle_max - self.angle_min) / self.angle_increment
        )

        # ── Calibration state ────────────────────────────────────────────────
        self.delta_z: float = 0.0          # depth bias offset (metres)
        self.is_calibrated: bool = False
        self._cal_samples: list[float] = []  # accumulate N Δz samples

        # ── TF2 ─────────────────────────────────────────────────────────────
        self.tf_buffer   = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # ── QoS profiles ─────────────────────────────────────────────────────
        lidar_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        camera_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # ── Subscribers + time synchronizer ──────────────────────────────────
        self.rgbd_sub  = Subscriber(self, PointCloud2, rgbd_topic,  qos_profile=camera_qos)
        self.lidar_sub = Subscriber(self, LaserScan,   lidar_topic, qos_profile=lidar_qos)

        self.sync = ApproximateTimeSynchronizer(
            [self.rgbd_sub, self.lidar_sub],
            queue_size=30,
            slop=1.0,
        )
        self.sync.registerCallback(self.fusion_callback)

        # ── Publisher ────────────────────────────────────────────────────────
        self.pub = self.create_publisher(LaserScan, "/fused_scan", 10)

        self.get_logger().info(
            f"DepthFusionNode2D started | bins={self.num_bins} | "
            f"auto_calibrate={self.auto_calibrate}"
        )

    # ── Internal TF helper ───────────────────────────────────────────────────

    def _get_camera_transform(
        self, frame_id: str, stamp
    ) -> tuple[np.ndarray, np.ndarray] | None:
        """
        Look up the transform from camera frame → target_frame.

        Returns
        -------
        (R, translation) or None on failure.
        R           : np.ndarray shape (3, 3)
        translation : np.ndarray shape (3,)
        """
        try:
            tf = self.tf_buffer.lookup_transform(
                self.target_frame,
                frame_id,
                stamp,
                timeout=rclpy.duration.Duration(seconds=0.1),
            )
        except Exception as e:
            self.get_logger().warn(f"TF lookup failed ({frame_id} → "
                                   f"{self.target_frame}): {e}")
            return None

        t = tf.transform.translation
        r = tf.transform.rotation
        if not all(np.isfinite([t.x, t.y, t.z, r.x, r.y, r.z, r.w])):
            self.get_logger().warn("TF contains NaN/Inf — skipping frame")
            return None

        R = quat_to_rot([r.x, r.y, r.z, r.w])
        translation = np.array([t.x, t.y, t.z], dtype=np.float32)
        return R, translation

    # ── Calibration trigger ───────────────────────────────────────────────────

    def _try_calibrate(
        self,
        lidar_msg: LaserScan,
        cloud_msg: PointCloud2,
        R: np.ndarray,
        translation: np.ndarray,
    ) -> None:
        """
        Attempt one calibration sample. Accumulates up to
        `calibration_attempts` samples and averages them to set delta_z.
        """
        delta = run_depth_bias_calibration(
            lidar_msg=lidar_msg,
            cloud_msg=cloud_msg,
            R=R,
            translation=translation,
            frontal_half_angle_deg=self.frontal_angle,
            min_height=self.min_height,
            max_height=self.max_height,
        )

        if delta is None:
            self.get_logger().debug(
                "Calibration sample failed (not enough wall points). "
                "Point the robot at a flat wall."
            )
            return

        self._cal_samples.append(delta)
        remaining = self.cal_attempts - len(self._cal_samples)
        self.get_logger().info(
            f"Calibration sample {len(self._cal_samples)}/{self.cal_attempts}: "
            f"Δz = {delta:+.4f} m  ({remaining} more needed)"
        )

        if len(self._cal_samples) >= self.cal_attempts:
            self.delta_z = float(np.mean(self._cal_samples))
            self.is_calibrated = True
            std = float(np.std(self._cal_samples))
            self.get_logger().info(
                f"✅ Depth-bias calibration complete | "
                f"Δz = {self.delta_z:+.4f} m ± {std:.4f} m"
            )

    # ── RGBD projection ───────────────────────────────────────────────────────

    def _rgbd_to_bins(
        self,
        cloud_msg: PointCloud2,
        R: np.ndarray,
        translation: np.ndarray,
    ) -> np.ndarray:
        """
        Project the (possibly bias-corrected) RGBD cloud onto the output
        angular grid.

        Returns
        -------
        bins : (num_bins,) float32, inf = no return in that bin
        """
        bins = np.full(self.num_bins, np.inf, dtype=np.float32)

        try:
            pts = point_cloud2.read_points_numpy(
                cloud_msg, field_names=["x", "y", "z"], skip_nans=True
            )

            if pts is None or len(pts) == 0:
                return bins

            # Unpack structured array
            if pts.dtype.names is not None:
                pts = np.column_stack(
                    [pts["x"], pts["y"], pts["z"]]
                ).astype(np.float32)
            else:
                pts = pts.astype(np.float32)

            # Remove non-finite rows
            pts = pts[np.isfinite(pts).all(axis=1)]
            if len(pts) == 0:
                return bins

            # ── Transform to base_footprint ──────────────────────────────────
            pts = (R @ pts.T).T + translation

            # ── Depth-bias correction (Z-axis shift) ─────────────────────────
            if self.is_calibrated:
                pts = apply_depth_bias(pts, self.delta_z, axis=2)

            # ── Height filter ────────────────────────────────────────────────
            mask_h = (pts[:, 2] >= self.min_height) & (pts[:, 2] <= self.max_height)
            pts = pts[mask_h]
            if len(pts) == 0:
                return bins

            # ── 2D projection ────────────────────────────────────────────────
            angles = np.arctan2(pts[:, 1], pts[:, 0])
            ranges = np.linalg.norm(pts[:, :2], axis=1)

            valid = (
                (angles >= self.angle_min)
                & (angles <= self.angle_max)
                & (ranges >= self.range_min)
                & (ranges <= self.range_max)
            )
            angles = angles[valid]
            ranges = ranges[valid]

            # ── Bin assignment (vectorised min-per-bin) ───────────────────────
            bin_idx = np.clip(
                ((angles - self.angle_min) / self.angle_increment).astype(int),
                0,
                self.num_bins - 1,
            )
            np.minimum.at(bins, bin_idx, ranges)

        except Exception as e:
            self.get_logger().error(f"RGBD projection failed: {e}")

        return bins

    # ── LiDAR re-binning ──────────────────────────────────────────────────────

    def _lidar_to_bins(self, scan_msg: LaserScan) -> np.ndarray:
        """
        Re-bin the raw LaserScan onto the output angular grid.
        Handles lidars whose native angle_min/increment differ from ours.

        Returns
        -------
        bins : (num_bins,) float32, inf = no return in that bin
        """
        bins = np.full(self.num_bins, np.inf, dtype=np.float32)

        raw = np.array(scan_msg.ranges, dtype=np.float32)
        raw_angles = scan_msg.angle_min + np.arange(len(raw)) * scan_msg.angle_increment

        # Wrap to [-π, π]
        raw_angles = np.where(raw_angles > np.pi, raw_angles - 2 * np.pi, raw_angles)

        valid = (
            np.isfinite(raw)
            & (raw >= scan_msg.range_min)
            & (raw <= scan_msg.range_max)
            & (raw_angles >= self.angle_min)
            & (raw_angles <= self.angle_max)
        )
        raw_angles = raw_angles[valid]
        raw = raw[valid]

        bin_idx = np.clip(
            ((raw_angles - self.angle_min) / self.angle_increment).astype(int),
            0,
            self.num_bins - 1,
        )
        np.minimum.at(bins, bin_idx, raw)
        return bins

    # ── Main callback ─────────────────────────────────────────────────────────

    def fusion_callback(self, rgbd_msg: PointCloud2, lidar_msg: LaserScan) -> None:
        """
        Synchronized callback: calibrate (if needed) then fuse and publish.
        """
        # ── Get camera → base_footprint transform ─────────────────────────────
        tf_result = self._get_camera_transform(
            rgbd_msg.header.frame_id, rgbd_msg.header.stamp
        )
        if tf_result is None:
            return
        R, translation = tf_result

        # ── Calibration (runs until is_calibrated == True) ────────────────────
        if self.auto_calibrate and not self.is_calibrated:
            self._try_calibrate(lidar_msg, rgbd_msg, R, translation)
            # Still publish fused data during calibration using delta_z = 0
            # so navigation is not blocked while the robot faces the wall.

        # ── Build per-bin range arrays ─────────────────────────────────────────
        rgbd_bins  = self._rgbd_to_bins(rgbd_msg, R, translation)
        lidar_bins = self._lidar_to_bins(lidar_msg)

        # ── Range-adaptive fusion ─────────────────────────────────────────────
        fused = range_adaptive_fusion(
            lidar_bins=lidar_bins,
            rgbd_bins=rgbd_bins,
            range_max=self.range_max,
            range_min=self.range_min,
            sigma_rgbd=self.sigma_rgbd,
            alpha=self.alpha,
        )

        # ── Build and publish LaserScan ───────────────────────────────────────
        out = LaserScan()
        out.header.stamp    = lidar_msg.header.stamp
        out.header.frame_id = self.target_frame
        out.angle_min       = self.angle_min
        out.angle_max       = self.angle_max
        out.angle_increment = self.angle_increment
        out.time_increment  = 0.0
        out.scan_time       = lidar_msg.scan_time
        out.range_min       = self.range_min
        out.range_max       = self.range_max
        out.ranges          = fused.tolist()
        out.intensities     = []

        self.pub.publish(out)

        if not self.is_calibrated:
            self.get_logger().debug(
                f"Publishing pre-calibration fused scan (Δz = 0). "
                f"Samples collected: {len(self._cal_samples)}/{self.cal_attempts}"
            )


# ════════════════════════════════════════════════════════════════════════════
#  ENTRY POINT
# ════════════════════════════════════════════════════════════════════════════


def main(args=None):
    rclpy.init(args=args)
    node = DepthFusionNode2D()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()