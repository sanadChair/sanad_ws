"""Robust depth sampling from a RealSense aligned depth image (float32, metres)."""

import numpy as np

_MIN_DEPTH  = 0.15   # D4xx reliable near limit (m)
_MAX_DEPTH  = 10.0   # D4xx reliable far  limit (m)
_MIN_PIXELS = 5      # minimum valid pixels to trust the estimate


def sample_depth(
    depth_m: np.ndarray,
    x1: float, y1: float, x2: float, y2: float,
    shrink: float = 0.60,
    kernel: int = 7,
) -> tuple[float, bool]:
    """
    Return (median_depth_m, valid) for a bounding box region.

    The inner `shrink` fraction of the box is sampled to avoid the
    silhouette-edge bleed that structured-light cameras produce.
    Degenerate or tiny boxes fall back to a centroid kernel window.
    """
    h, w = depth_m.shape

    margin_x = (x2 - x1) * (1 - shrink) / 2
    margin_y = (y2 - y1) * (1 - shrink) / 2

    r0 = int(max(0, y1 + margin_y))
    r1 = int(min(h, y2 - margin_y))
    c0 = int(max(0, x1 + margin_x))
    c1 = int(min(w, x2 - margin_x))

    if r1 <= r0 or c1 <= c0:
        # Box too small after shrinking — fall back to centroid kernel
        cx, cy = int((x1 + x2) / 2), int((y1 + y2) / 2)
        half   = kernel // 2
        r0, r1 = max(0, cy - half), min(h, cy + half + 1)
        c0, c1 = max(0, cx - half), min(w, cx + half + 1)

    patch = depth_m[r0:r1, c0:c1]
    valid = patch[(patch > _MIN_DEPTH) & (patch < _MAX_DEPTH)]

    if valid.size < _MIN_PIXELS:
        return 0.0, False

    return float(np.median(valid)), True