import numpy as np
import rclpy
from sensor_msgs.msg import Image, CameraInfo
from vision_msgs.msg import Detection3DArray
from cv_bridge import CvBridge
from image_geometry import PinholeCameraModel
from message_filters import ApproximateTimeSynchronizer, Subscriber
from .detector_base import DetectorBase
from .detection_visualizer import DetectionVisualizer
import cv2

class YoloDepthDetector(DetectorBase):
    """
    Detects humans using yolov26n on the RGB stream, then back-projects
    each detection's centroid through the aligned depth image to get a
    3-D point in the camera frame.

    Params:
        model_name   – yolov26n checkpoint  (default 'yolov26n.pt')
        conf_thresh  – minimum confidence   (default 0.50)
        depth_scale  – metres per depth unit (default 1.0 for Gazebo float32)
        sync_slop    – time sync tolerance in seconds (default 0.05)
        debug_viz    – show OpenCV debug window (default False)
    """

    def __init__(self):
        try:
            from ultralytics import YOLO as _YOLO
            self._YOLO = _YOLO
        except ImportError:
            raise RuntimeError(
                'ultralytics not found — pip install ultralytics')

        super().__init__('yolo_depth_detector')

    # ── Setup ────────────────────────────────────────────────────────────────

    def _setup_subscribers(self):
        self.declare_parameter('model_name',  'yolo26n.pt')
        self.declare_parameter('conf_thresh',  0.50)
        self.declare_parameter('depth_scale',  1.0)
        self.declare_parameter('sync_slop',    0.05)
        self.declare_parameter('debug_viz',    False)

        model_name = self.get_parameter('model_name').value
        self.get_logger().info(f'Loading YOLO model: {model_name}')
        self._model     = self._YOLO(model_name)
        self.get_logger().info(f'Loaded YOLO model: {model_name}')
        self._bridge    = CvBridge()
        self._cam_model = PinholeCameraModel()
        self._got_info  = False
        self._viz       = DetectionVisualizer()

        self.create_subscription(
            CameraInfo, '/realsense/camera_info',
            self._on_info, 1)

        slop = self.get_parameter('sync_slop').value
        self._rgb_sub   = Subscriber(self, Image,
                                     '/realsense/image',       qos_profile=self.QOS)
        self._depth_sub = Subscriber(self, Image,
                                     '/realsense/depth_image', qos_profile=self.QOS)
        self._sync = ApproximateTimeSynchronizer(
            [self._rgb_sub, self._depth_sub], queue_size=5, slop=slop)
        self._sync.registerCallback(self._on_sync)

    def _on_info(self, msg: CameraInfo):
        if not self._got_info:
            self._cam_model.fromCameraInfo(msg)
            self._got_info = True
            self.get_logger().info('Camera model received.')

    def _on_sync(self, rgb_msg: Image, depth_msg: Image):
        self.get_logger().info('_on_sync fired!')   # ← add this
        if not self._got_info:
            self.get_logger().warn('Waiting for camera info...')
            return
        self._publish(self._detect(rgb_msg, depth_msg))

    # ── Core detection ───────────────────────────────────────────────────────

    def _detect(self, rgb_msg: Image, depth_msg: Image) -> Detection3DArray:
        conf_thresh = self.get_parameter('conf_thresh').value
        depth_scale = self.get_parameter('depth_scale').value
        debug_viz   = self.get_parameter('debug_viz').value

        rgb_raw = self._bridge.imgmsg_to_cv2(rgb_msg, desired_encoding='passthrough')
        depth = self._bridge.imgmsg_to_cv2(depth_msg, '32FC1')

        rgb = self._bridge.imgmsg_to_cv2(rgb_msg, 'rgb8')
        rgb = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)


        results = self._model(rgb, classes=[0], verbose=False)
        self.get_logger().info(f'YOLO found {len(results[0].boxes)} boxes')
        arr = Detection3DArray()
        arr.header.frame_id = depth_msg.header.frame_id

        if debug_viz:
            self._viz.begin_frame(rgb)

        for box in results[0].boxes:
            conf = float(box.conf[0])
            if conf < conf_thresh:
                continue

            x1, y1, x2, y2 = box.xyxy[0].tolist()
            cx = int((x1 + x2) / 2)
            cy = int((y1 + y2) / 2)

            h, w = depth.shape
            cx = np.clip(cx, 0, w - 1)
            cy = np.clip(cy, 0, h - 1)

            patch = depth[max(0, cy - 3):cy + 4, max(0, cx - 3):cx + 4]
            valid_depths = patch[np.isfinite(patch) & (patch > 0.1)]
            if len(valid_depths) == 0:
                if debug_viz:
                    self._viz.draw_no_depth(x1, y1, x2, y2, cx, cy, conf)
                continue

            z = float(np.median(valid_depths)) * depth_scale

            ray    = self._cam_model.projectPixelTo3dRay((cx, cy))
            x3d    = ray[0] * z
            y3d    = ray[1] * z
            fx     = self._cam_model.fx()
            size_x = (x2 - x1) / fx * z
            size_y = (y2 - y1) / fx * z

            arr.detections.append(
                self.make_detection(x3d, y3d, z, score=conf,
                                    size_x=size_x, size_y=size_y, size_z=1.7))

            if debug_viz:
                self._viz.draw_detection(x1, y1, x2, y2,
                                         cx, cy, conf, z, x3d, y3d)

        if debug_viz:
            self._viz.draw_hud(depth, len(arr.detections))
            self._viz.show()

        return arr


def main(args=None):
    rclpy.init(args=args)
    node = YoloDepthDetector()
    try:
        rclpy.spin(node)
    finally:
        node._viz.close()
        node.destroy_node()
        rclpy.shutdown()