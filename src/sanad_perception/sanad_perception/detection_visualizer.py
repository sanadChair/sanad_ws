import numpy as np
import cv2


class DetectionVisualizer:
    """
    Renders YOLO + depth detection debug information onto an OpenCV canvas.

    Usage
    -----
    viz = DetectionVisualizer()
    viz.begin_frame(rgb_bgr_array)          # call once per frame
    viz.draw_detection(...)                  # call for every valid detection
    viz.draw_no_depth(...)                   # call when depth sampling failed
    viz.draw_hud(depth_array, n_published)  # call once after all detections
    viz.show()                              # renders the window
    """

    # ── Colours (BGR) ────────────────────────────────────────────────────────
    COLOUR_BOX    = (0, 255, 0)      # green  — valid detection box
    COLOUR_PATCH  = (0, 165, 255)    # orange — depth-patch sample region
    COLOUR_TEXT   = (255, 255, 255)  # white  — label foreground
    COLOUR_SHADOW = (0, 0, 0)        # black  — label drop-shadow
    COLOUR_NODEPTH = (0, 0, 255)     # red    — no-depth detection

    FONT       = cv2.FONT_HERSHEY_SIMPLEX
    FONT_SCALE = 0.45
    FONT_THICK = 1

    WINDOW_NAME = 'YoloDepthDetector — debug'

    def __init__(self):
        self._canvas: np.ndarray | None = None

    # ── Frame lifecycle ──────────────────────────────────────────────────────

    def begin_frame(self, rgb: np.ndarray) -> None:
        """Copy the RGB image to start a fresh canvas for this frame."""
        self._canvas = rgb.copy()

    def show(self) -> None:
        """Push the current canvas to the named OpenCV window."""
        if self._canvas is None:
            return
        cv2.imshow(self.WINDOW_NAME, self._canvas)
        cv2.waitKey(1)

    def close(self) -> None:
        """Destroy the debug window (call on node shutdown)."""
        cv2.destroyWindow(self.WINDOW_NAME)

    # ── Per-detection drawing ────────────────────────────────────────────────

    def draw_detection(
        self,
        x1: float, y1: float, x2: float, y2: float,
        cx: int, cy: int,
        conf: float,
        z: float, x3d: float, y3d: float,
    ) -> None:
        """
        Overlay a successfully back-projected detection.

        Draws:
          • bounding box
          • 7 × 7 depth-patch sample region
          • centroid crosshair
          • top label  : confidence + depth
          • bottom label: 3-D camera-frame position
        """
        self._assert_canvas()
        ix1, iy1, ix2, iy2 = int(x1), int(y1), int(x2), int(y2)

        cv2.rectangle(self._canvas, (ix1, iy1), (ix2, iy2),
                      self.COLOUR_BOX, 2)

        cv2.rectangle(self._canvas,
                      (max(0, cx - 3), max(0, cy - 3)),
                      (cx + 3, cy + 3),
                      self.COLOUR_PATCH, 1)

        cv2.drawMarker(self._canvas, (cx, cy), self.COLOUR_BOX,
                       cv2.MARKER_CROSS, 12, 2)

        self._put_label(f'person {conf:.2f} | z={z:.2f}m', ix1, iy1 - 6)
        self._put_label(f'3D ({x3d:+.2f}, {y3d:+.2f}, {z:.2f}) m',
                        ix1, iy2 + 16)

    def draw_no_depth(
        self,
        x1: float, y1: float, x2: float, y2: float,
        cx: int, cy: int,
        conf: float,
    ) -> None:
        """
        Highlight a detection that was dropped because depth sampling failed.

        Draws a red box, a tilted-cross at the centroid, and a 'NO DEPTH' label.
        """
        self._assert_canvas()
        ix1, iy1, ix2, iy2 = int(x1), int(y1), int(x2), int(y2)

        cv2.rectangle(self._canvas, (ix1, iy1), (ix2, iy2),
                      self.COLOUR_NODEPTH, 2)
        cv2.drawMarker(self._canvas, (cx, cy), self.COLOUR_NODEPTH,
                       cv2.MARKER_TILTED_CROSS, 12, 2)
        self._put_label(f'person {conf:.2f} | NO DEPTH', ix1, iy1 - 6)

    # ── Frame-level HUD ──────────────────────────────────────────────────────

    def draw_hud(self, depth: np.ndarray, n_published: int) -> None:
        """
        Render a semi-transparent stats panel in the top-left corner.

        Shows: published detection count, valid-depth pixel percentage,
        and the min/max depth range for the frame.
        """
        self._assert_canvas()

        valid_mask = np.isfinite(depth) & (depth > 0.1)
        valid_px   = int(np.sum(valid_mask))
        total_px   = depth.size
        depth_pct  = 100.0 * valid_px / total_px if total_px else 0.0

        if valid_px:
            d_min = float(depth[valid_mask].min())
            d_max = float(depth[valid_mask].max())
            range_str = f'{d_min:.2f} – {d_max:.2f} m'
        else:
            range_str = 'N/A'

        lines = [
            f'detections published : {n_published}',
            f'depth valid pixels   : {depth_pct:.1f}%',
            f'depth range          : {range_str}',
        ]

        panel_h = 16 + 20 * len(lines)
        panel_w = 370
        overlay = self._canvas.copy()
        cv2.rectangle(overlay, (0, 0), (panel_w, panel_h), (0, 0, 0), -1)
        cv2.addWeighted(overlay, 0.45, self._canvas, 0.55, 0, self._canvas)

        for i, line in enumerate(lines):
            cv2.putText(self._canvas, line, (6, 18 + 20 * i),
                        self.FONT, 0.48, self.COLOUR_TEXT, 1, cv2.LINE_AA)

    # ── Internal helpers ─────────────────────────────────────────────────────

    def _put_label(self, text: str, x: int, y: int) -> None:
        """Draw text with a one-pixel dark drop-shadow for contrast."""
        cv2.putText(self._canvas, text, (x + 1, y + 1),
                    self.FONT, self.FONT_SCALE, self.COLOUR_SHADOW,
                    2, cv2.LINE_AA)
        cv2.putText(self._canvas, text, (x, y),
                    self.FONT, self.FONT_SCALE, self.COLOUR_TEXT,
                    self.FONT_THICK, cv2.LINE_AA)

    def _assert_canvas(self) -> None:
        if self._canvas is None:
            raise RuntimeError(
                'DetectionVisualizer: call begin_frame() before drawing.')