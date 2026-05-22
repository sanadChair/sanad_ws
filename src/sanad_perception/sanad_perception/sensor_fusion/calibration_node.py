"""
calibration_node.py
===================
ROS 2 node that performs the one-time RANSAC depth-bias calibration
and publishes the resulting Δz on ``/depth_bias``.

Subscribes
----------
  /realsense/points   (sensor_msgs/PointCloud2)   — configurable via param
  /scan               (sensor_msgs/LaserScan)      — configurable via param

Publishes
---------
  /depth_bias         (std_msgs/Float32)
      Published with TRANSIENT_LOCAL (latched) durability so that the
      fusion node receives the value even if it starts after calibration
      completes.  Re-published after every averaged batch so the fusion
      node always has the latest estimate.

  /calibration_status (std_msgs/String)
      Human-readable status string; useful for dashboards / rqt.

Usage
-----
  # Run standalone (robot facing a flat wall):
  ros2 run sanad_fusion calibration_node

  # Override number of calibration samples:
  ros2 run sanad_fusion calibration_node \
      --ros-args -p calibration_attempts:=10
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
from std_msgs.msg import Float32, String
from tf2_ros import Buffer, TransformListener

import json
from pathlib import Path
from datetime import datetime, timezone

from .fusion_utils import quat_to_rot, run_depth_bias_calibration


class CalibrationNode(Node):
    """
    Accumulates *N* RANSAC depth-bias samples and publishes their mean as
    ``std_msgs/Float32`` on ``/depth_bias``.

    Once ``calibration_attempts`` samples have been collected the node
    considers itself done and stops processing new frames (it keeps
    spinning so the latched topic remains live).
    """

    # ── Lifecycle ────────────────────────────────────────────────────────────

    def __init__(self):
        super().__init__("calibration_node")

        # ── Parameters ───────────────────────────────────────────────────────
        self.declare_parameter("rgbd_topic",              "/realsense/points")
        self.declare_parameter("lidar_topic",             "/scan")
        self.declare_parameter("target_frame",            "base_footprint")
        self.declare_parameter("calibration_attempts",    5)
        self.declare_parameter("frontal_half_angle_deg",  20.0)
        self.declare_parameter("min_height",              0.1)
        self.declare_parameter("max_height",              2.0)

        rgbd_topic      = self.get_parameter("rgbd_topic").value
        lidar_topic     = self.get_parameter("lidar_topic").value
        self.target_frame   = self.get_parameter("target_frame").value
        self.cal_attempts   = self.get_parameter("calibration_attempts").value
        self.frontal_angle  = self.get_parameter("frontal_half_angle_deg").value
        self.min_height     = self.get_parameter("min_height").value
        self.max_height     = self.get_parameter("max_height").value

        # ── State ─────────────────────────────────────────────────────────────
        self._cal_samples: list[float] = []
        self.delta_z: float = 0.0
        self.is_calibrated: bool = False

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
        # TRANSIENT_LOCAL = latched: late-joining subscribers get the last msg
        latched_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # ── Publishers ───────────────────────────────────────────────────────
        self.bias_pub   = self.create_publisher(Float32, "/depth_bias",         latched_qos)
        self.status_pub = self.create_publisher(String,  "/calibration_status", latched_qos)

        # ── Subscribers + synchronizer ────────────────────────────────────────
        self.rgbd_sub  = Subscriber(self, PointCloud2, rgbd_topic,  qos_profile=camera_qos)
        self.lidar_sub = Subscriber(self, LaserScan,   lidar_topic, qos_profile=lidar_qos)

        self.sync = ApproximateTimeSynchronizer(
            [self.rgbd_sub, self.lidar_sub],
            queue_size=30,
            slop=1.0,
        )
        self.sync.registerCallback(self._sync_callback)

        self._publish_status(
            f"Waiting for calibration data "
            f"(0/{self.cal_attempts} samples). "
            "Point the robot at a flat wall."
        )
        self.get_logger().info(
            f"CalibrationNode started | target_frame={self.target_frame} | "
            f"attempts needed={self.cal_attempts}"
        )

    # ── TF helper ────────────────────────────────────────────────────────────

    def _get_camera_transform(
        self, frame_id: str, stamp
    ) -> tuple[np.ndarray, np.ndarray] | None:
        """
        Look up camera frame → target_frame.

        Returns
        -------
        (R, translation) or None on failure.
        """
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

    # ── Calibration logic ─────────────────────────────────────────────────────

    def _sync_callback(
        self, rgbd_msg: PointCloud2, lidar_msg: LaserScan
    ) -> None:
        """Synchronized RGBD + LiDAR callback — collects calibration samples."""
        if self.is_calibrated:
            return  # nothing left to do

        tf_result = self._get_camera_transform(
            rgbd_msg.header.frame_id, rgbd_msg.header.stamp
        )
        if tf_result is None:
            return
        R, translation = tf_result

        self._collect_sample(lidar_msg, rgbd_msg, R, translation)

    def _collect_sample(
        self,
        lidar_msg: LaserScan,
        cloud_msg: PointCloud2,
        R: np.ndarray,
        translation: np.ndarray,
    ) -> None:
        """
        Attempt one calibration sample.  Averages and finalises after
        ``calibration_attempts`` successful samples.
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
                "Calibration sample failed — not enough wall points detected."
            )
            return

        self._cal_samples.append(delta)
        n   = len(self._cal_samples)
        rem = self.cal_attempts - n
        msg = (
            f"Sample {n}/{self.cal_attempts}: Δz = {delta:+.4f} m "
            f"({'done' if rem == 0 else f'{rem} more needed'})"
        )
        self.get_logger().info(msg)
        # self.get_logger().info(f"  (z_gt={z_gt:.4f} m, z_meas={z_meas:.4f} m)")
        self._publish_status(msg)

        if n >= self.cal_attempts:
            self._finalise_calibration()

    def _finalise_calibration(self) -> None:
        """Average samples, publish Δz, mark node as done."""
        self.delta_z      = float(np.mean(self._cal_samples))
        self.is_calibrated = True
        std = float(np.std(self._cal_samples))

        self.get_logger().info(
            f"✅ Calibration complete | "
            f"Δz = {self.delta_z:+.4f} m ± {std:.4f} m"
        )

        bias_msg = Float32()
        bias_msg.data = self.delta_z
        self.bias_pub.publish(bias_msg)

        self._publish_status(
            f"DONE | Δz = {self.delta_z:+.4f} m ± {std:.4f} m"
        )

    def _publish_status(self, text: str) -> None:
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)


# ── Entry point ───────────────────────────────────────────────────────────────


def main(args=None):
    rclpy.init(args=args)
    node = CalibrationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()