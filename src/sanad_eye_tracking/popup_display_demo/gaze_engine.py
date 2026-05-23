#!/usr/bin/env python3
"""
gaze_engine.py – Standalone eye/gaze tracker (no ROS dependency).

Extracts the gaze projection logic from eye_tracker_node.py and exposes it
through a simple GazeEngine class that yields (gaze_x, gaze_y, eye_state)
per frame.
"""

import os
import numpy as np
import cv2
import mediapipe as mp
from mediapipe.tasks import python as mp_python
from mediapipe.tasks.python import vision as mp_vision
from collections import deque

# ── MediaPipe landmark indices (478-point model) ────────────────────
LEFT_IRIS  = [469, 470, 471, 472]
RIGHT_IRIS = [474, 475, 476, 477]

LEFT_EYE_LEFT_CORNER  = 33
LEFT_EYE_RIGHT_CORNER = 133
RIGHT_EYE_LEFT_CORNER  = 362
RIGHT_EYE_RIGHT_CORNER = 263

LEFT_EYE_TOP    = 159
LEFT_EYE_BOTTOM = 145
RIGHT_EYE_TOP    = 386
RIGHT_EYE_BOTTOM = 374


def _iris_centre(landmarks, iris_indices, img_w, img_h):
    xs = [landmarks[i].x * img_w for i in iris_indices]
    ys = [landmarks[i].y * img_h for i in iris_indices]
    return np.mean(xs), np.mean(ys)


def _eye_corners(landmarks, left_idx, right_idx, img_w, img_h):
    lx = landmarks[left_idx].x * img_w
    ly = landmarks[left_idx].y * img_h
    rx = landmarks[right_idx].x * img_w
    ry = landmarks[right_idx].y * img_h
    return (lx, ly), (rx, ry)


def _eye_aspect_ratio(landmarks, top_idx, bottom_idx, left_idx, right_idx,
                       img_w, img_h):
    top    = np.array([landmarks[top_idx].x * img_w,    landmarks[top_idx].y * img_h])
    bottom = np.array([landmarks[bottom_idx].x * img_w, landmarks[bottom_idx].y * img_h])
    left   = np.array([landmarks[left_idx].x * img_w,   landmarks[left_idx].y * img_h])
    right  = np.array([landmarks[right_idx].x * img_w,  landmarks[right_idx].y * img_h])
    vert = np.linalg.norm(top - bottom)
    horiz = np.linalg.norm(left - right)
    if horiz == 0:
        return 0.0
    return vert / horiz


class GazeEngine:
    """
    Standalone gaze tracker.  Call .process_frame() each tick to get
    (gaze_x, gaze_y, eye_state, frame).
    """

    def __init__(
        self,
        camera_index=0,
        model_path=None,
        h_range=0.040,
        v_range=0.020,
        h_offset=0.000,
        v_offset=-0.015,
        smoothing=0.3,
        ear_threshold=0.20,
    ):
        # ── Resolve model ───────────────────────────────────────────
        if model_path is None:
            model_path = os.path.join(
                os.path.dirname(__file__), "..", "config", "face_landmarker.task"
            )
        if not os.path.isfile(model_path):
            raise FileNotFoundError(
                f"FaceLandmarker model not found at: {model_path}\n"
                "Download: wget -O <path>/face_landmarker.task "
                "https://storage.googleapis.com/mediapipe-models/"
                "face_landmarker/face_landmarker/float16/latest/face_landmarker.task"
            )

        # ── Params ──────────────────────────────────────────────────
        self.h_range = h_range
        self.v_range = v_range
        self.h_offset = h_offset
        self.v_offset = v_offset
        self.smoothing = smoothing
        self.ear_thresh = ear_threshold

        # ── Camera ──────────────────────────────────────────────────
        self.cap = cv2.VideoCapture(camera_index)
        if not self.cap.isOpened():
            raise RuntimeError(f"Cannot open camera index {camera_index}")

        # ── MediaPipe ───────────────────────────────────────────────
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
        self._frame_ts = 0
        self._smoothed_x = 0.5
        self._smoothed_y = 0.5
        self._gaze_history = deque(maxlen=7)

    # ────────────────────────────────────────────────────────────────
    def process_frame(self):
        """
        Grab one frame and return (gaze_x, gaze_y, eye_state, frame).
        gaze_x/y are in [0.0, 1.0].  eye_state is "open"/"blink"/"closed".
        Returns None if frame grab fails.
        """
        ret, frame = self.cap.read()
        if not ret:
            return None

        frame = cv2.flip(frame, 1)
        h, w, _ = frame.shape
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb)
        self._frame_ts += 33  # ~30 fps

        result = self.landmarker.detect_for_video(mp_image, self._frame_ts)

        gaze_x = self._smoothed_x
        gaze_y = self._smoothed_y
        eye_state = "open"

        if result.face_landmarks and len(result.face_landmarks) > 0:
            lm = result.face_landmarks[0]

            # ── Gaze direction ──────────────────────────────────────
            l_iris = _iris_centre(lm, LEFT_IRIS, w, h)
            r_iris = _iris_centre(lm, RIGHT_IRIS, w, h)

            l_left, l_right = _eye_corners(
                lm, LEFT_EYE_LEFT_CORNER, LEFT_EYE_RIGHT_CORNER, w, h
            )
            r_left, r_right = _eye_corners(
                lm, RIGHT_EYE_LEFT_CORNER, RIGHT_EYE_RIGHT_CORNER, w, h
            )

            def _horiz_ratio(iris, left_c, right_c):
                eye_w = right_c[0] - left_c[0]
                if eye_w == 0:
                    return 0.0
                return ((iris[0] - left_c[0]) / eye_w) - 0.5

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

            norm_h = (h_ratio - self.h_offset) / self.h_range
            norm_v = (v_ratio - self.v_offset) / self.v_range
            norm_h = np.sign(norm_h) * (abs(norm_h) ** 1.2)
            norm_v = np.sign(norm_v) * (abs(norm_v) ** 1.2)
            norm_h = max(-1.0, min(1.0, norm_h))
            norm_v = max(-1.0, min(1.0, norm_v))
            raw_x = (norm_h + 1.0) / 2.0
            raw_y = (norm_v + 1.0) / 2.0

            # ── EAR ────────────────────────────────────────────────
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
                if self._prev_eye_state == "open":
                    eye_state = "blink"
                else:
                    eye_state = "closed"
                if len(self._gaze_history) > 0:
                    self._smoothed_x, self._smoothed_y = self._gaze_history[0]
            else:
                eye_state = "open"
                self._smoothed_x = (self.smoothing * raw_x) + (
                    (1.0 - self.smoothing) * self._smoothed_x
                )
                self._smoothed_y = (self.smoothing * raw_y) + (
                    (1.0 - self.smoothing) * self._smoothed_y
                )
                self._gaze_history.append((self._smoothed_x, self._smoothed_y))

            gaze_x = self._smoothed_x
            gaze_y = self._smoothed_y
            self._prev_eye_state = eye_state

        return gaze_x, gaze_y, eye_state, frame

    # ────────────────────────────────────────────────────────────────
    def release(self):
        self.cap.release()
        self.landmarker.close()
