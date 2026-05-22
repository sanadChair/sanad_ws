"""
fusion_node.py
==============
ROS 2 node that fuses a RealSense RGBD PointCloud2 and a raw LiDAR
LaserScan into a single 2D LaserScan published on ``/fused_scan``.

Subscribes
----------
  /realsense/points   (sensor_msgs/PointCloud2)   — configurable via param
  /scan               (sensor_msgs/LaserScan)      — configurable via param
  /depth_bias         (std_msgs/Float32)
      Written by ``calibration_node`` with TRANSIENT_LOCAL durability.
      The fusion node uses whatever value is available; it defaults to 0.0
      (no correction) until a value arrives, so navigation is never blocked
      while waiting for calibration.

Publishes
---------
  /fused_scan         (sensor_msgs/LaserScan)

Usage
-----
  # Fusion only (assumes calibration already published /depth_bias):
  ros2 run sanad_fusion fusion_node

  # Fusion with manual bias override (skip calibration node entirely):
  ros2 run sanad_fusion fusion_node --ros-args -p manual_delta_z:=0.05
"""

from __future__ import annotations

from click import Parameter
from pathlib import Path

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
from std_msgs.msg import Float32
from tf2_ros import Buffer, TransformListener
import json
from ament_index_python.packages import get_package_share_directory


from .fusion_utils import (
    apply_depth_bias,
    quat_to_rot,
    range_adaptive_fusion,
)


class FusionNode(Node):
    """
    Synchronized RGBD + LiDAR → fused 2D LaserScan.

    Depth-bias correction (Δz) is received from ``/depth_bias``; the node
    works correctly with Δz = 0 before calibration arrives.
    """

    # ── Lifecycle ────────────────────────────────────────────────────────────

    def __init__(self):
        super().__init__("fusion_node")

        # ── Output grid parameters ────────────────────────────────────────────
        self.declare_parameter("angle_min",        -3.14159)
        self.declare_parameter("angle_max",         3.14159)
        self.declare_parameter("angle_increment",   0.00581)   # ~0.333 °
        self.declare_parameter("range_min",         0.1)
        self.declare_parameter("range_max",        12.0)

        # ── Height filter ─────────────────────────────────────────────────────
        self.declare_parameter("min_height",  0.1)
        self.declare_parameter("max_height",  2.0)

        # ── Fusion parameters ─────────────────────────────────────────────────
        self.declare_parameter("sigma_rgbd",  1.5)
        self.declare_parameter("alpha",       1.0)

        # ── Topics / frames ───────────────────────────────────────────────────
        self.declare_parameter("rgbd_topic",    "/realsense/points")
        self.declare_parameter("lidar_topic",   "/scan")
        self.declare_parameter("target_frame",  "base_footprint")

        # ── Optional manual bias (overrides /depth_bias subscriber) ───────────
        # Set to a non-NaN value to bypass the calibration node entirely.
        self.declare_parameter("manual_delta_z", float("nan"))

        # ── Read params ───────────────────────────────────────────────────────
        self.angle_min       = self.get_parameter("angle_min").value
        self.angle_max       = self.get_parameter("angle_max").value
        self.angle_increment = self.get_parameter("angle_increment").value
        self.range_min       = self.get_parameter("range_min").value
        self.range_max       = self.get_parameter("range_max").value
        self.min_height      = self.get_parameter("min_height").value
        self.max_height      = self.get_parameter("max_height").value
        self.sigma_rgbd      = self.get_parameter("sigma_rgbd").value
        self.alpha           = self.get_parameter("alpha").value
        self.target_frame    = self.get_parameter("target_frame").value
        rgbd_topic           = self.get_parameter("rgbd_topic").value
        lidar_topic          = self.get_parameter("lidar_topic").value
        manual_delta_z       = self.get_parameter("manual_delta_z").value

        self.num_bins = int(
            (self.angle_max - self.angle_min) / self.angle_increment
        )

        self.calibration_file = Path(
                    get_package_share_directory("sanad_perception")
                ).parents[3] / "src" / "sanad_perception" / "sanad_perception" / "sensor_fusion" / "calibration.json"

        # ── Depth-bias state ──────────────────────────────────────────────────
        # 1. Try manual override
        if not np.isnan(manual_delta_z):
            self.delta_z = float(manual_delta_z)
            self.get_logger().info(
                f"Manual depth bias override: Δz = {self.delta_z:+.4f} m"
            )
        # 2. Else try loading from calibration file as a cold-start fallback
        else:
            loaded = self._load_bias_from_file()
            if loaded is not None:
                self.delta_z = loaded
                self.get_logger().info(
                    f"Loaded Δz={self.delta_z:+.4f} m from "
                    f"{self.calibration_file}"
                )
            else:
                self.delta_z = 0.0
                self.get_logger().warn(
                    f"No calibration file found at {self.calibration_file} — "
                    "running with Δz = 0.0 until /depth_bias arrives."
                )



        # ── TF2 ──────────────────────────────────────────────────────────────
        self.tf_buffer   = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # ── QoS ──────────────────────────────────────────────────────────────
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
        # Match calibration_node's TRANSIENT_LOCAL publisher
        bias_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # ── Subscribers ───────────────────────────────────────────────────────
        self.create_subscription(
            Float32, "/depth_bias", self._bias_callback, bias_qos
        )

        self.rgbd_sub  = Subscriber(self, PointCloud2, rgbd_topic,  qos_profile=camera_qos)
        self.lidar_sub = Subscriber(self, LaserScan,   lidar_topic, qos_profile=lidar_qos)

        self.sync = ApproximateTimeSynchronizer(
            [self.rgbd_sub, self.lidar_sub],
            queue_size=30,
            slop=1.0,
        )
        self.sync.registerCallback(self._fusion_callback)

        # ── Publisher ─────────────────────────────────────────────────────────
        self.pub = self.create_publisher(LaserScan, "/fused_scan", 10)

        self.get_logger().info(
            f"FusionNode started | bins={self.num_bins} | "
            f"target_frame={self.target_frame} | "
            f"initial Δz={self.delta_z:+.4f} m"
        )

    def _load_bias_from_file(self) -> float | None:
        """Load Δz from disk if the calibration file exists."""
        if not self.calibration_file.exists():
            return None

        try:
            with self.calibration_file.open("r") as fh:
                data = json.load(fh)

            val = float(data["delta_z"])
            if not np.isfinite(val):
                raise ValueError("non-finite value")

            return val

        except Exception as exc:
            self.get_logger().error(
                f"Failed to load {self.calibration_file}: {exc}"
            )
            return None

    # ── Bias callback ─────────────────────────────────────────────────────────

    def _bias_callback(self, msg: Float32) -> None:
        """Update depth-bias correction whenever calibration_node publishes."""
        if np.isfinite(msg.data):
            old = self.delta_z
            self.delta_z = float(msg.data)
            self.get_logger().info(
                f"Depth bias updated: {old:+.4f} → {self.delta_z:+.4f} m"
            )

    # ── TF helper ────────────────────────────────────────────────────────────

    def _get_camera_transform(
        self, frame_id: str, stamp
    ) -> tuple[np.ndarray, np.ndarray] | None:
        try:
            tf = self.tf_buffer.lookup_transform(
                self.target_frame,
                frame_id,
                stamp,
                timeout=rclpy.duration.Duration(seconds=0.1),
            )
        except Exception as exc:
            self.get_logger().warn(
                f"TF lookup failed ({frame_id} → {self.target_frame}): {exc}"
            )
            return None

        t = tf.transform.translation
        r = tf.transform.rotation
        if not all(np.isfinite([t.x, t.y, t.z, r.x, r.y, r.z, r.w])):
            self.get_logger().warn("TF contains NaN/Inf — skipping frame")
            return None

        R = quat_to_rot([r.x, r.y, r.z, r.w])
        translation = np.array([t.x, t.y, t.z], dtype=np.float32)
        return R, translation

    # ── RGBD projection ───────────────────────────────────────────────────────

    def _rgbd_to_bins(
        self,
        cloud_msg: PointCloud2,
        R: np.ndarray,
        translation: np.ndarray,
    ) -> np.ndarray:
        """
        Project the (bias-corrected) RGBD cloud onto the output angular grid.

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

            if pts.dtype.names is not None:
                pts = np.column_stack(
                    [pts["x"], pts["y"], pts["z"]]
                ).astype(np.float32)
            else:
                pts = pts.astype(np.float32)

            pts = pts[np.isfinite(pts).all(axis=1)]
            if len(pts) == 0:
                return bins

            # Transform → base_footprint
            pts = (R @ pts.T).T + translation

            # Depth-bias correction
            if self.delta_z != 0.0:
                pts = apply_depth_bias(pts, self.delta_z, axis=2)

            # Height filter
            mask_h = (pts[:, 2] >= self.min_height) & (pts[:, 2] <= self.max_height)
            pts = pts[mask_h]
            if len(pts) == 0:
                return bins

            # 2-D projection
            angles = np.arctan2(pts[:, 1], pts[:, 0])
            ranges = np.linalg.norm(pts[:, :2], axis=1)

            valid = (
                (angles >= self.angle_min) & (angles <= self.angle_max)
                & (ranges >= self.range_min) & (ranges <= self.range_max)
            )
            angles = angles[valid]
            ranges = ranges[valid]

            bin_idx = np.clip(
                ((angles - self.angle_min) / self.angle_increment).astype(int),
                0,
                self.num_bins - 1,
            )
            np.minimum.at(bins, bin_idx, ranges)

        except Exception as exc:
            self.get_logger().error(f"RGBD projection failed: {exc}")

        return bins

    # ── LiDAR re-binning ──────────────────────────────────────────────────────

    def _lidar_to_bins(self, scan_msg: LaserScan) -> np.ndarray:
        """
        Re-bin a raw LaserScan onto the output angular grid.

        Returns
        -------
        bins : (num_bins,) float32, inf = no return in that bin
        """
        bins = np.full(self.num_bins, np.inf, dtype=np.float32)

        raw = np.array(scan_msg.ranges, dtype=np.float32)
        raw_angles = (
            scan_msg.angle_min
            + np.arange(len(raw)) * scan_msg.angle_increment
        )
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

    def _fusion_callback(
        self, rgbd_msg: PointCloud2, lidar_msg: LaserScan
    ) -> None:
        """Synchronized callback: project → fuse → publish."""
        tf_result = self._get_camera_transform(
            rgbd_msg.header.frame_id, rgbd_msg.header.stamp
        )
        if tf_result is None:
            return
        R, translation = tf_result

        rgbd_bins  = self._rgbd_to_bins(rgbd_msg, R, translation)
        lidar_bins = self._lidar_to_bins(lidar_msg)

        fused = range_adaptive_fusion(
            lidar_bins=lidar_bins,
            rgbd_bins=rgbd_bins,
            range_max=self.range_max,
            range_min=self.range_min,
            sigma_rgbd=self.sigma_rgbd,
            alpha=self.alpha,
        )

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


# ── Entry point ───────────────────────────────────────────────────────────────


def main(args=None):
    rclpy.init(args=args)
    node = FusionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()