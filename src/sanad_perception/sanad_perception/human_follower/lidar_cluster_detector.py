# sanad_perception/lidar_cluster_detector.py
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from vision_msgs.msg import Detection3DArray
from .detector_base import DetectorBase


class LidarClusterDetector(DetectorBase):
    """
    Detects humans from 2-D LiDAR by Euclidean clustering.

    Tunable ROS 2 params:
        cluster_dist  – gap (m) that splits clusters        (default 0.20)
        min_points    – minimum points to keep a cluster    (default 4)
        max_width     – max cluster width in m (walls wider) (default 0.75)
        min_dist      – ignore returns closer than this m   (default 0.30)
        max_dist      – ignore returns farther than this m  (default 6.00)
    """

    def __init__(self):
        super().__init__('lidar_cluster_detector')

        self.declare_parameter('cluster_dist', 0.20)
        self.declare_parameter('min_points',   4)
        self.declare_parameter('max_width',    0.75)
        self.declare_parameter('min_dist',     0.30)
        self.declare_parameter('max_dist',     6.00)

    # ── DetectorBase interface ───────────────────────────────────────────────

    def _setup_subscribers(self):
        self.create_subscription(
            LaserScan, '/scan', self._on_scan, self.QOS)

    def _detect(self, msg: LaserScan) -> Detection3DArray:
        p = self._get_params()
        pts = self._scan_to_points(msg, p)
        if len(pts) == 0:
            return Detection3DArray()

        clusters = self._cluster(pts, p['cluster_dist'])
        arr = Detection3DArray()
        arr.header.frame_id = msg.header.frame_id

        for cl in clusters:
            if len(cl) < p['min_points']:
                continue
            span = np.linalg.norm(cl.max(axis=0) - cl.min(axis=0))
            if span > p['max_width']:
                continue                    # too wide → wall or furniture
            cx, cy = cl.mean(axis=0)
            arr.detections.append(
                self.make_detection(float(cx), float(cy), 0.0))

        return arr

    # ── Internal helpers ─────────────────────────────────────────────────────

    def _on_scan(self, msg: LaserScan):
        self._publish(self._detect(msg))

    def _get_params(self):
        return {
            'cluster_dist': self.get_parameter('cluster_dist').value,
            'min_points':   int(self.get_parameter('min_points').value),
            'max_width':    self.get_parameter('max_width').value,
            'min_dist':     self.get_parameter('min_dist').value,
            'max_dist':     self.get_parameter('max_dist').value,
        }

    @staticmethod
    def _scan_to_points(msg: LaserScan, p: dict) -> np.ndarray:
        n = len(msg.ranges)
        angles = np.linspace(msg.angle_min, msg.angle_max, n)
        ranges = np.array(msg.ranges, dtype=np.float32)

        valid = (
            np.isfinite(ranges) &
            (ranges >= max(msg.range_min, p['min_dist'])) &
            (ranges <= min(msg.range_max, p['max_dist']))
        )
        r = ranges[valid]
        a = angles[valid]
        return np.column_stack([r * np.cos(a), r * np.sin(a)])

    @staticmethod
    def _cluster(pts: np.ndarray, gap: float):
        """Simple sequential clustering — O(N), good enough for <1000 pts."""
        if len(pts) == 0:
            return []
        clusters, current = [], [pts[0]]
        for i in range(1, len(pts)):
            if np.linalg.norm(pts[i] - pts[i - 1]) <= gap:
                current.append(pts[i])
            else:
                clusters.append(np.array(current))
                current = [pts[i]]
        clusters.append(np.array(current))
        return clusters


def main(args=None):
    rclpy.init(args=args)
    node = LidarClusterDetector()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()