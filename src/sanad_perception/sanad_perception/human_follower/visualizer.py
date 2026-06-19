"""Debug visualisation for the detector/tracker node."""

import numpy as np
import cv2


def _track_colour(track_id: int) -> tuple[int, int, int]:
    rng = np.random.default_rng(seed=track_id * 2654435761 & 0xFFFFFFFF)
    return tuple(int(c) for c in rng.integers(60, 220, 3))


def draw_tracks(frame: np.ndarray, tracks) -> np.ndarray:
    """Return a copy of `frame` annotated with bounding boxes and depth info."""
    viz = frame.copy()

    for tp in tracks:
        x1, y1, x2, y2 = (int(v) for v in tp.bbox)
        colour = _track_colour(tp.track_id)

        cv2.rectangle(viz, (x1, y1), (x2, y2), colour, 2)

        # Depth bar on the left edge: green (far) → red (close)
        if tp.depth_valid:
            ratio = min(1.0, tp.depth_m / 5.0)
            bar_colour = (int(255 * ratio), int(255 * (1 - ratio)), 0)
            bar_top = y2 - int((y2 - y1) * (1.0 - ratio))
            cv2.rectangle(viz, (x1 - 6, bar_top), (x1 - 2, y2), bar_colour, -1)

        # Label
        depth_str = f'{tp.depth_m:.2f}m' if tp.depth_valid else '---'
        label = f'ID:{tp.track_id}  {depth_str}  {tp.confidence:.2f}'
        (lw, lh), bl = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.45, 1)
        ly = max(y1, lh + 8)
        cv2.rectangle(viz, (x1, ly - lh - bl - 4), (x1 + lw + 4, ly), colour, -1)
        cv2.putText(viz, label, (x1 + 2, ly - bl - 2),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1, cv2.LINE_AA)

        # Centroid dot
        cx, cy = int(tp.centroid_x), int(tp.centroid_y)
        cv2.circle(viz, (cx, cy), 4, colour, -1)
        cv2.circle(viz, (cx, cy), 4, (255, 255, 255), 1)

    n = len(tracks)
    cv2.putText(viz, f'{n} person{"s" if n != 1 else ""}',
                (10, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                (200, 200, 200), 1, cv2.LINE_AA)
    return viz