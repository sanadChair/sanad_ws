#!/usr/bin/env python3
"""
eye_tracker_node.py – Real-time eye / gaze tracker using MediaPipe FaceLandmarker
(Tasks API, mediapipe >= 0.10).

Published topics
----------------
/eye_tracking/gaze_direction  (std_msgs/String)
    One of: "north", "south", "east", "west", "center"

/eye_tracking/eye_state  (std_msgs/String)
    One of: "open", "blink", "closed"

The node grabs frames from a webcam, runs the MediaPipe FaceLandmarker to
extract 478 facial landmarks (including iris 468-477), and derives:
  • Horizontal / vertical gaze direction  (iris centre relative to eye corners)
  • Eye Aspect Ratio (EAR) to detect blinks / prolonged closure
"""

import os
import numpy as np

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

try:
    import cv2
except ImportError:
    raise ImportError(
        "OpenCV is required.  Install with:  pip install opencv-python"
    )

try:
    import mediapipe as mp
    from mediapipe.tasks import python as mp_python
    from mediapipe.tasks.python import vision as mp_vision
except ImportError:
    raise ImportError(
        "MediaPipe >= 0.10 is required.  Install with:  pip install mediapipe"
    )


# ── MediaPipe landmark indices (478-point model) ────────────────────
# Iris landmarks
LEFT_IRIS  = [469, 470, 471, 472]   # subject's right eye (camera left)
RIGHT_IRIS = [474, 475, 476, 477]   # subject's left eye  (camera right)

# Eye corner landmarks for gaze ratio
# Left eye (camera perspective)
LEFT_EYE_LEFT_CORNER  = 33
LEFT_EYE_RIGHT_CORNER = 133

# Right eye (camera perspective)
RIGHT_EYE_LEFT_CORNER  = 362
RIGHT_EYE_RIGHT_CORNER = 263

# EAR landmarks  (top / bottom / left / right of each eye)
LEFT_EYE_TOP    = 159
LEFT_EYE_BOTTOM = 145
RIGHT_EYE_TOP    = 386
RIGHT_EYE_BOTTOM = 374


def _iris_centre(landmarks, iris_indices, img_w, img_h):
    """Return (x, y) pixel coords of the iris centre."""
    xs = [landmarks[i].x * img_w for i in iris_indices]
    ys = [landmarks[i].y * img_h for i in iris_indices]
    return np.mean(xs), np.mean(ys)


def _eye_corners(landmarks, left_idx, right_idx, img_w, img_h):
    """Return pixel coords of left and right corners of an eye."""
    lx = landmarks[left_idx].x * img_w
    ly = landmarks[left_idx].y * img_h
    rx = landmarks[right_idx].x * img_w
    ry = landmarks[right_idx].y * img_h
    return (lx, ly), (rx, ry)


def _eye_aspect_ratio(landmarks, top_idx, bottom_idx, left_idx, right_idx,
                       img_w, img_h):
    """Compute EAR ≈ vertical_dist / horizontal_dist."""
    top    = np.array([landmarks[top_idx].x * img_w,    landmarks[top_idx].y * img_h])
    bottom = np.array([landmarks[bottom_idx].x * img_w, landmarks[bottom_idx].y * img_h])
    left   = np.array([landmarks[left_idx].x * img_w,   landmarks[left_idx].y * img_h])
    right  = np.array([landmarks[right_idx].x * img_w,  landmarks[right_idx].y * img_h])
    vert = np.linalg.norm(top - bottom)
    horiz = np.linalg.norm(left - right)
    if horiz == 0:
        return 0.0
    return vert / horiz


class EyeTrackerNode(Node):
    """Publishes gaze direction and eye state at a configurable rate."""

    def __init__(self):
        super().__init__("eye_tracker")

        # ── Parameters ──────────────────────────────────────────────
        self.declare_parameter("camera_index", 0)
        self.declare_parameter("publish_rate", 30.0)
        self.declare_parameter("gaze_horizontal_threshold", 0.015)
        self.declare_parameter("gaze_vertical_threshold", 0.012)
        self.declare_parameter("ear_threshold", 0.20)
        self.declare_parameter("show_debug_window", True)
        self.declare_parameter("model_path", "")

        self.cam_idx    = self.get_parameter("camera_index").value
        self.rate_hz    = self.get_parameter("publish_rate").value
        self.h_thresh   = self.get_parameter("gaze_horizontal_threshold").value
        self.v_thresh   = self.get_parameter("gaze_vertical_threshold").value
        self.ear_thresh = self.get_parameter("ear_threshold").value
        self.show_debug = self.get_parameter("show_debug_window").value
        model_path_param = self.get_parameter("model_path").value

        # ── Resolve model path ──────────────────────────────────────
        if model_path_param:
            model_path = model_path_param
        else:
            # Look next to the config yaml (installed share dir)
            try:
                from ament_index_python.packages import get_package_share_directory
                pkg_share = get_package_share_directory("sanad_eye_tracking")
                model_path = os.path.join(pkg_share, "config", "face_landmarker.task")
            except Exception:
                model_path = os.path.join(
                    os.path.dirname(__file__), "..", "config", "face_landmarker.task"
                )

        if not os.path.isfile(model_path):
            self.get_logger().error(
                f"FaceLandmarker model not found at: {model_path}\n"
                "Download it with:\n"
                "  wget -O <path>/face_landmarker.task "
                "https://storage.googleapis.com/mediapipe-models/"
                "face_landmarker/face_landmarker/float16/latest/face_landmarker.task"
            )
            raise FileNotFoundError(f"Model not found: {model_path}")

        self.get_logger().info(f"Loading FaceLandmarker model from: {model_path}")

        # ── Publishers ──────────────────────────────────────────────
        self.gaze_pub = self.create_publisher(String, "/eye_tracking/gaze_direction", 10)
        self.eye_pub  = self.create_publisher(String, "/eye_tracking/eye_state", 10)

        # ── Camera ──────────────────────────────────────────────────
        self.cap = cv2.VideoCapture(self.cam_idx)
        if not self.cap.isOpened():
            self.get_logger().error(f"Cannot open camera index {self.cam_idx}")
            raise RuntimeError("Camera not available")
        self.get_logger().info(f"Camera {self.cam_idx} opened successfully")

        # ── MediaPipe FaceLandmarker (Tasks API) ────────────────────
        base_options = mp_python.BaseOptions(model_asset_path=model_path)
        options = mp_vision.FaceLandmarkerOptions(
            base_options=base_options,
            running_mode=mp_vision.RunningMode.VIDEO,
            num_faces=1,
            min_face_detection_confidence=0.5,
            min_face_presence_confidence=0.5,
            min_tracking_confidence=0.5,
        )
        self.landmarker = mp_vision.FaceLandmarker.create_from_options(options)

        # ── State ───────────────────────────────────────────────────
        self._prev_eye_state = "open"
        self._frame_ts = 0  # monotonic frame timestamp in ms for VIDEO mode

        # ── Timer ───────────────────────────────────────────────────
        period = 1.0 / self.rate_hz
        self.create_timer(period, self._timer_callback)
        self.get_logger().info("EyeTrackerNode started  ✓")

    # ────────────────────────────────────────────────────────────────
    def _timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn("Failed to grab frame", throttle_duration_sec=5.0)
            return

        frame = cv2.flip(frame, 1)  # mirror for natural interaction
        h, w, _ = frame.shape
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

        # Convert to MediaPipe Image
        mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb)
        self._frame_ts += int(1000 / self.rate_hz)  # advance timestamp

        # Detect landmarks
        result = self.landmarker.detect_for_video(mp_image, self._frame_ts)

        gaze_dir = "center"
        eye_state = "open"

        if result.face_landmarks and len(result.face_landmarks) > 0:
            lm = result.face_landmarks[0]  # list of NormalizedLandmark

            # ── Gaze direction ──────────────────────────────────────
            l_iris = _iris_centre(lm, LEFT_IRIS, w, h)
            r_iris = _iris_centre(lm, RIGHT_IRIS, w, h)

            l_left, l_right = _eye_corners(
                lm, LEFT_EYE_LEFT_CORNER, LEFT_EYE_RIGHT_CORNER, w, h
            )
            r_left, r_right = _eye_corners(
                lm, RIGHT_EYE_LEFT_CORNER, RIGHT_EYE_RIGHT_CORNER, w, h
            )

            # Normalised horizontal offset  (-0.5 … +0.5)
            def _horiz_ratio(iris, left_c, right_c):
                eye_w = right_c[0] - left_c[0]
                if eye_w == 0:
                    return 0.0
                return ((iris[0] - left_c[0]) / eye_w) - 0.5

            # Normalised vertical offset
            def _vert_ratio(iris, left_c, right_c):
                mid_y = (left_c[1] + right_c[1]) / 2.0
                eye_w = right_c[0] - left_c[0]
                if eye_w == 0:
                    return 0.0
                return (iris[1] - mid_y) / eye_w

            h_ratio = (_horiz_ratio(l_iris, l_left, l_right) +
                       _horiz_ratio(r_iris, r_left, r_right)) / 2.0
            v_ratio = (_vert_ratio(l_iris, l_left, l_right) +
                       _vert_ratio(r_iris, r_left, r_right)) / 2.0

            # Determine dominant direction
            if abs(v_ratio) > abs(h_ratio):
                if v_ratio < -self.v_thresh:
                    gaze_dir = "north"
                elif v_ratio > self.v_thresh:
                    gaze_dir = "south"
            else:
                if h_ratio < -self.h_thresh:
                    gaze_dir = "west"
                elif h_ratio > self.h_thresh:
                    gaze_dir = "east"

            # ── Eye Aspect Ratio (blink / closed detection) ─────────
            l_ear = _eye_aspect_ratio(
                lm, LEFT_EYE_TOP, LEFT_EYE_BOTTOM,
                LEFT_EYE_LEFT_CORNER, LEFT_EYE_RIGHT_CORNER, w, h
            )
            r_ear = _eye_aspect_ratio(
                lm, RIGHT_EYE_TOP, RIGHT_EYE_BOTTOM,
                RIGHT_EYE_LEFT_CORNER, RIGHT_EYE_RIGHT_CORNER, w, h
            )
            avg_ear = (l_ear + r_ear) / 2.0

            if avg_ear < self.ear_thresh:
                # Eyes are shut right now
                if self._prev_eye_state == "open":
                    eye_state = "blink"   # transition: open → closed
                else:
                    eye_state = "closed"  # still closed
            else:
                eye_state = "open"

            self._prev_eye_state = eye_state

            # ── Debug drawing ───────────────────────────────────────
            if self.show_debug:
                # Draw iris circles
                for ix, iy in [l_iris, r_iris]:
                    cv2.circle(frame, (int(ix), int(iy)), 3, (0, 255, 0), -1)
                # Draw info text
                cv2.putText(frame, f"Gaze: {gaze_dir}",
                            (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8,
                            (0, 255, 255), 2)
                cv2.putText(frame, f"EAR: {avg_ear:.3f}  State: {eye_state}",
                            (10, 65), cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                            (0, 200, 200), 2)
                cv2.putText(frame, f"H: {h_ratio:.4f}  V: {v_ratio:.4f}",
                            (10, 95), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                            (200, 200, 0), 1)

        # ── Publish ─────────────────────────────────────────────────
        gaze_msg = String()
        gaze_msg.data = gaze_dir
        self.gaze_pub.publish(gaze_msg)

        eye_msg = String()
        eye_msg.data = eye_state
        self.eye_pub.publish(eye_msg)

        # ── Show debug window ───────────────────────────────────────
        if self.show_debug:
            cv2.imshow("SANAD Eye Tracker", frame)
            cv2.waitKey(1)

    # ────────────────────────────────────────────────────────────────
    def destroy_node(self):
        self.cap.release()
        if self.show_debug:
            cv2.destroyAllWindows()
        self.landmarker.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = EyeTrackerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
