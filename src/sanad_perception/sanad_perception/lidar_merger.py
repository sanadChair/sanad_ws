import math

import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformException, TransformListener


class LidarMerger(Node):

    def __init__(self):
        super().__init__("lidar_merger")

        self.declare_parameter("target_frame", "base_link")

        self.sub1 = self.create_subscription(
            LaserScan, "/scan_left_filtered", self.cb1, 10
        )

        self.sub2 = self.create_subscription(
            LaserScan, "/scan_right_filtered", self.cb2, 10
        )

        self.pub = self.create_publisher(LaserScan, "/scan", 10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.scan1 = None
        self.scan2 = None

    def cb1(self, msg):
        self.scan1 = msg
        self.merge()

    def cb2(self, msg):
        self.scan2 = msg
        self.merge()

    def merge(self):
        if self.scan1 is None or self.scan2 is None:
            return

        target_frame = self.get_parameter("target_frame").value
        scan1_points = self._scan_to_points_in_frame(self.scan1, target_frame)
        scan2_points = self._scan_to_points_in_frame(self.scan2, target_frame)

        if scan1_points is None or scan2_points is None:
            return

        merged = LaserScan()
        merged.header.stamp = self.get_clock().now().to_msg()
        merged.header.frame_id = target_frame

        merged.angle_min = -math.pi
        merged.angle_increment = self.scan1.angle_increment
        merged.angle_max = (
            merged.angle_min
            + (self._bin_count(merged.angle_increment) - 1) * merged.angle_increment
        )

        merged.range_min = min(self.scan1.range_min, self.scan2.range_min)
        merged.range_max = max(self.scan1.range_max, self.scan2.range_max)

        merged.ranges = self._merge_points(
            [scan1_points, scan2_points],
            merged.angle_min,
            merged.angle_increment,
            merged.range_min,
            merged.range_max,
        )

        self.pub.publish(merged)

    def _scan_to_points_in_frame(self, scan, target_frame):
        try:
            transform = self.tf_buffer.lookup_transform(
                target_frame,
                scan.header.frame_id,
                scan.header.stamp,
                timeout=Duration(seconds=0.2),
            )
        except TransformException as exc:
            self.get_logger().warn(
                f"Unable to transform {scan.header.frame_id} to {target_frame}: {exc}"
            )
            return None

        ranges = np.asarray(scan.ranges, dtype=np.float32)
        angles = (
            scan.angle_min
            + np.arange(len(ranges), dtype=np.float32) * scan.angle_increment
        )
        valid = np.isfinite(ranges)
        valid &= ranges >= scan.range_min
        valid &= ranges <= scan.range_max

        if not np.any(valid):
            return np.empty((0, 2), dtype=np.float32)

        x = ranges[valid] * np.cos(angles[valid])
        y = ranges[valid] * np.sin(angles[valid])
        return self._transform_xy(x, y, transform)

    @staticmethod
    def _transform_xy(x, y, transform):
        translation = transform.transform.translation
        rotation = transform.transform.rotation

        yaw = math.atan2(
            2.0 * (rotation.w * rotation.z + rotation.x * rotation.y),
            1.0 - 2.0 * (rotation.y * rotation.y + rotation.z * rotation.z),
        )
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)

        x_out = cos_yaw * x - sin_yaw * y + translation.x
        y_out = sin_yaw * x + cos_yaw * y + translation.y
        return np.column_stack((x_out, y_out))

    def _merge_points(
        self, point_sets, angle_min, angle_increment, range_min, range_max
    ):
        bin_count = self._bin_count(angle_increment)
        merged = np.full(bin_count, np.inf, dtype=np.float32)

        for points in point_sets:
            if len(points) == 0:
                continue

            angles = np.arctan2(points[:, 1], points[:, 0])
            ranges = np.hypot(points[:, 0], points[:, 1])

            bins = np.floor((angles - angle_min) / angle_increment).astype(np.int32)
            valid = (bins >= 0) & (bins < bin_count)
            valid &= np.isfinite(ranges)
            valid &= ranges >= range_min
            valid &= ranges <= range_max

            bins = bins[valid]
            ranges = ranges[valid]

            for idx, distance in zip(bins, ranges):
                if distance < merged[idx]:
                    merged[idx] = distance

        return merged.tolist()

    def _bin_count(self, angle_increment):
        return max(1, int(round((2.0 * math.pi) / angle_increment)))


def main():
    rclpy.init()
    node = LidarMerger()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
