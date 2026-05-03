# sanad_perception/detector_base.py
from abc import ABC, abstractmethod
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from vision_msgs.msg import Detection3DArray
from visualization_msgs.msg import MarkerArray
from sensor_msgs.msg import Image


class DetectorBase(Node, ABC):
    """
    Abstract base for all human detectors with optional visualization.

    Subclasses must implement _setup_subscribers() and _detect().
    They call self._publish(arr) when a Detection3DArray is ready.
    """

    QOS = QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
    )

    def __init__(self, node_name: str):
        super().__init__(node_name)
        self._pub = self.create_publisher(
            Detection3DArray, '/detections', 10)
        self.get_logger().info(f'{node_name} ready — publishing /detections')
        
        # Visualization publishers
        self._vis_pub = self.create_publisher(Image, '/detections/image', 10)
        self._markers_pub = self.create_publisher(MarkerArray, '/detections/markers', 10)
        self._depth_vis_pub = self.create_publisher(Image, '/detections/depth_overlay', 10)
        
        self._setup_subscribers()

    @abstractmethod
    def _setup_subscribers(self):
        """Create all sensor subscriptions here."""

    @abstractmethod
    def _detect(self, *args) -> Detection3DArray:
        """Run detection on raw sensor data, return Detection3DArray."""

    def _publish(self, arr: Detection3DArray, vis_image=None, marker_array=None, depth_overlay=None):
        """Publish detections and optional visualization."""
        arr.header.stamp = self.get_clock().now().to_msg()
        self._pub.publish(arr)
        
        if vis_image is not None:
            self._vis_pub.publish(vis_image)
        if marker_array is not None:
            self._markers_pub.publish(marker_array)
        if depth_overlay is not None:
            self._depth_vis_pub.publish(depth_overlay)

    # ── Shared helper: build a Detection3D from a 3-D point ─────────────────
    @staticmethod
    def make_detection(x: float, y: float, z: float,
                       score: float = 0.9,
                       size_x=0.5, size_y=0.5, size_z=1.7):
        from vision_msgs.msg import (Detection3D,
                                     ObjectHypothesisWithPose)
        det = Detection3D()
        det.bbox.center.position.x = x
        det.bbox.center.position.y = y
        det.bbox.center.position.z = z
        det.bbox.size.x = size_x
        det.bbox.size.y = size_y
        det.bbox.size.z = size_z
        hyp = ObjectHypothesisWithPose()
        hyp.hypothesis.class_id = 'person'
        hyp.hypothesis.score = score
        det.results.append(hyp)
        return det