"""RealSense RGB+Depth → YOLO26 + ByteTrack → TrackedPersonArray."""

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import message_filters
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from ultralytics import YOLO
import cv2 as cv
import threading
from collections import deque
import torch

from sanad_interfaces.msg import TrackedPerson, TrackedPersonArray
from .depth_utils import sample_depth
from .visualizer import draw_tracks


_SENSOR_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=5,
)
_DEBUG_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=5,
)


_DEFAULTS = {
    'model_path':           "yolo26n.pt",
    'confidence_threshold':  0.40,
    'iou_threshold':         0.45,
    'depth_bbox_shrink':     0.60,
    'depth_kernel_size':     7,
    'sync_slop':             0.15,
    'debug_viz':             True,
    'rgb_topic':            "/realsense/image",
    'depth_topic':          "/realsense/depth_image",
    'device': 'cuda:0',
}


class DetectorTrackerNode(Node):

    def __init__(self):
        super().__init__('detector_tracker_node')

        for name, default in _DEFAULTS.items():
            self.declare_parameter(name, default)

        def p(name):
            return self.get_parameter(name).value

        self._conf   = p('confidence_threshold')
        self._iou    = p('iou_threshold')
        self._shrink = p('depth_bbox_shrink')
        self._kernel = p('depth_kernel_size')
        self._debug  = p('debug_viz')

        self.device  = p("device")
        if 'cuda' in self.device and not torch.cuda.is_available():
            self.get_logger().warn(
                f'CUDA requested but not available — falling back to CPU. '
                f'Reason: driver/PyTorch CUDA version mismatch.'
            )
            self.device = 'cpu'


        self._model  = self._load_model(p('model_path'))

        self._frame_lock  = threading.Condition()
        self._frame_queue: deque = deque(maxlen=1)
        self._infer_thread = threading.Thread(target=self._infer_loop, daemon=True)
        self._infer_thread.start()

        self._bridge = CvBridge()

        rgb_sub = message_filters.Subscriber(
            self, Image, p('rgb_topic'), qos_profile=_SENSOR_QOS)
        depth_sub = message_filters.Subscriber(
            self, Image, p('depth_topic'), qos_profile=_SENSOR_QOS)

        self._sync = message_filters.ApproximateTimeSynchronizer(
            [rgb_sub, depth_sub],
            queue_size=10,   # was 5
            slop=p('sync_slop'),
            allow_headerless=False,
        )
        self._sync.registerCallback(self._on_images)

        self._tracks_pub = self.create_publisher(TrackedPersonArray, '/person/tracks', 10)
        self._debug_pub  = (
            self.create_publisher(Image, '/person/debug_image', _DEBUG_QOS) if self._debug else None
        )

        self._sync_count = 0
        self._drop_timer = self.create_timer(10.0, self._log_sync_rate)

        self._last_cb_stamp = self.get_clock().now()
        self.create_timer(5.0, self._watchdog)

        self.get_logger().info('DetectorTrackerNode ready')

    def _load_model(self, path: str) -> YOLO:
        self.get_logger().info(f'Loading model: {path}')
        model = YOLO(path)
        model.track(
            np.zeros((480, 640, 3), dtype=np.uint8),
            classes=[0],
            tracker='bytetrack.yaml',
            persist=True,
            verbose=False,
        )
        self.get_logger().info('Model ready')
        return model
    
    def _log_sync_rate(self):
        self.get_logger().info(f'Sync callbacks in last 10s: {self._sync_count}')
        self._sync_count = 0
    
    def _watchdog(self):
        elapsed = (self.get_clock().now() - self._last_cb_stamp).nanoseconds * 1e-9
        if elapsed > 5.0:
            self.get_logger().warn(
                f'No sync callback for {elapsed:.1f}s — '
                'check camera topics or increase sync_slop'
            )

    def _infer_loop(self):
        """Dedicated inference thread — drains the queue and runs YOLO.
 
        This is the ONLY place _process() is called. _on_images() simply
        enqueues the frame pair; it never calls _process() itself, avoiding
        the double-processing bug where every frame was handled twice.
        """
        while rclpy.ok():
            with self._frame_lock:
                while not self._frame_queue:
                    self._frame_lock.wait(timeout=0.1)   # blocks until notify()
                    
                rgb_msg, depth_msg = self._frame_queue.pop()
            try:
                self._process(rgb_msg, depth_msg)
            except Exception as e:
                self.get_logger().error(
                    f'Unhandled error in inference loop: {e}',
                    throttle_duration_sec=2.0,
                )

    # ------------------------------------------------------------------ #
    # Sync callback — ENQUEUE ONLY, never call _process() here            #
    # ------------------------------------------------------------------ #
    def _on_images(self, rgb_msg: Image, depth_msg: Image):
        self._last_cb_stamp = self.get_clock().now()
        self._sync_count += 1
        with self._frame_lock:
            self._frame_queue.append((rgb_msg, depth_msg))
            self._frame_lock.notify()          # wake inference thread immediately

    def _process(self, rgb_msg: Image, depth_msg: Image):
        try:
            bgr     = self._bridge.imgmsg_to_cv2(rgb_msg,   'bgr8')
            depth_m = self._bridge.imgmsg_to_cv2(depth_msg, 'passthrough').astype(np.float32) 
        except Exception as e:
            self.get_logger().error(f'Decode error: {e}')
            return
    

        t0 = self.get_clock().now()
        results = self._model.track(
            bgr,
            classes=[0],
            conf=self._conf,
            iou=self._iou,
            tracker='bytetrack.yaml',
            persist=True,
            verbose=False,
            half=True,
            device=self.device,
        )
        dt = (self.get_clock().now() - t0).nanoseconds * 1e-6  # ns → ms

        self.get_logger().info(f'Inference: {dt:.1f} ms', throttle_duration_sec=2.0)


        out = TrackedPersonArray(header=rgb_msg.header, target_id=-1)
        r = results[0] if results else None

        if r is not None and r.boxes is not None and r.boxes.id is not None:
            out.tracks = [
                self._make_track(rgb_msg.header, r.boxes, i, depth_m)
                for i in range(len(r.boxes))
            ]

        self._tracks_pub.publish(out)
        self.get_logger().info(f"Published {out.tracks}")

        if self._debug_pub and (self._sync_count % 3 == 0): # publish every 3rd frame
            viz = draw_tracks(bgr, out.tracks)
            msg = self._bridge.cv2_to_imgmsg(viz, 'bgr8')
            msg.header = rgb_msg.header
            self._debug_pub.publish(msg)
            self.get_logger().info(
                f'Tracks: {len(out.tracks)}',
                throttle_duration_sec=2.0,
            )

    def _make_track(self, header, boxes, i: int, depth_m: np.ndarray) -> TrackedPerson:
        x1, y1, x2, y2 = boxes.xyxy[i].cpu().numpy().tolist()
        depth_val, valid = sample_depth(
            depth_m, x1, y1, x2, y2, self._shrink, self._kernel)

        return TrackedPerson(
            header     = header,
            track_id   = int(boxes.id[i].item()),
            confidence = float(boxes.conf[i].item()),
            bbox       = [x1, y1, x2, y2],
            centroid_x = (x1 + x2) / 2,
            centroid_y = (y1 + y2) / 2,
            depth_m    = depth_val,
            depth_valid = valid,
        )


def main(args=None):
    rclpy.init(args=args)
    node = DetectorTrackerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()