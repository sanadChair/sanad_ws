"""
fusion_utils.py
===============
Pure-numpy calibration and fusion utilities for the sanad depth-fusion
pipeline.  No ROS dependency — import and test freely in isolation.

Public API
----------
  quat_to_rot                 — quaternion → 3×3 rotation matrix
  ransac_wall_distance        — RANSAC line fit → distance to wall
  extract_lidar_wall_points   — frontal LaserScan arc → Cartesian points
  compute_rgbd_median_depth   — RGBD cloud → median forward depth
  run_depth_bias_calibration  — full calibration routine → Δz
  apply_depth_bias            — shift Z column of a point array
  range_adaptive_fusion       — fuse LiDAR + RGBD bin arrays

References
----------
  Delhibabu et al., "2D grid map creation based on RGBD-camera and
  LiDAR data", Scientific Reports 16:10591 (2026).
"""

from __future__ import annotations

import numpy as np

# ── sensor_msgs_py is only available inside a ROS environment; guard the
#    import so the rest of the module can still be used in plain pytest.
try:
    from sensor_msgs_py import point_cloud2 as _pc2

    def _read_points(cloud_msg):  # type: ignore[return]
        return _pc2.read_points_numpy(
            cloud_msg, field_names=["x", "y", "z"], skip_nans=True
        )

except ImportError:  # pragma: no cover
    _read_points = None  # type: ignore[assignment]


# ════════════════════════════════════════════════════════════════════════════
#  GEOMETRY
# ════════════════════════════════════════════════════════════════════════════


def quat_to_rot(q: list[float]) -> np.ndarray:
    """
    Convert a unit quaternion ``[x, y, z, w]`` to a 3×3 rotation matrix.

    Parameters
    ----------
    q : sequence of four floats — [x, y, z, w]

    Returns
    -------
    R : np.ndarray, shape (3, 3), dtype float32
    """
    x, y, z, w = q
    return np.array(
        [
            [1 - 2*(y*y + z*z),  2*(x*y - z*w),  2*(x*z + y*w)],
            [2*(x*y + z*w),  1 - 2*(x*x + z*z),  2*(y*z - x*w)],
            [2*(x*z - y*w),  2*(y*z + x*w),  1 - 2*(x*x + y*y)],
        ],
        dtype=np.float32,
    )


# ════════════════════════════════════════════════════════════════════════════
#  RANSAC WALL DETECTION
# ════════════════════════════════════════════════════════════════════════════


def ransac_wall_distance(
    points_xy: np.ndarray,
    n_iter: int = 150,
    inlier_threshold: float = 0.03,
    min_inliers: int = 8,
) -> float | None:
    """
    Fit a line  ``ax + by + c = 0``  to 2-D LiDAR points via RANSAC and
    return the perpendicular distance from the sensor origin to that line.

    Implements *Algorithm 2* from Delhibabu et al. (2026).

    Parameters
    ----------
    points_xy         : np.ndarray, shape (N, 2) — x/y in LiDAR frame
    n_iter            : number of RANSAC iterations
    inlier_threshold  : max point-to-line distance (m) to count as inlier
    min_inliers       : minimum inliers required to accept a model

    Returns
    -------
    perp_distance : float — distance from origin to best-fit wall, or
                    None  — if no valid model was found
    """
    if len(points_xy) < 2:
        return None

    best_inlier_count = 0
    best_distance: float | None = None

    for _ in range(n_iter):
        idx = np.random.choice(len(points_xy), 2, replace=False)
        p1, p2 = points_xy[idx[0]], points_xy[idx[1]]

        dx, dy = p2[0] - p1[0], p2[1] - p1[1]
        length = np.hypot(dx, dy)
        if length < 1e-6:
            continue

        # Unit-normal line equation: a·x + b·y + c = 0
        a =  dy / length
        b = -dx / length
        c = -(a * p1[0] + b * p1[1])

        point_dists = np.abs(a * points_xy[:, 0] + b * points_xy[:, 1] + c)
        inliers = int(np.sum(point_dists < inlier_threshold))

        if inliers > best_inlier_count:
            best_inlier_count = inliers
            # a,b are unit-normalised → sqrt(a²+b²) = 1
            best_distance = float(abs(c))

    return best_distance if best_inlier_count >= min_inliers else None


def extract_lidar_wall_points(
    ranges: np.ndarray,
    angle_min: float,
    angle_increment: float,
    frontal_half_angle_deg: float = 20.0,
    range_min: float = 0.1,
    range_max: float = 12.0,
) -> np.ndarray | None:
    """
    Extract Cartesian (x, y) points from the frontal arc of a LaserScan.

    Parameters
    ----------
    ranges                  : raw ranges array from LaserScan
    angle_min               : LaserScan.angle_min  (radians)
    angle_increment         : LaserScan.angle_increment  (radians)
    frontal_half_angle_deg  : half-width of frontal cone (degrees)
    range_min / range_max   : validity bounds (m)

    Returns
    -------
    points_xy : np.ndarray shape (N, 2), or None if fewer than 10 valid points
    """
    half_rad = np.deg2rad(frontal_half_angle_deg)
    angles = angle_min + np.arange(len(ranges)) * angle_increment
    # Wrap to [-π, π]
    angles = np.where(angles > np.pi, angles - 2 * np.pi, angles)

    valid = (
        np.isfinite(ranges)
        & (ranges >= range_min)
        & (ranges <= range_max)
        & (np.abs(angles) <= half_rad)
    )
    if np.sum(valid) < 10:
        return None

    xs = ranges[valid] * np.cos(angles[valid])
    ys = ranges[valid] * np.sin(angles[valid])
    return np.stack([xs, ys], axis=1).astype(np.float32)


# ════════════════════════════════════════════════════════════════════════════
#  RGBD DEPTH EXTRACTION
# ════════════════════════════════════════════════════════════════════════════


def compute_rgbd_median_depth(
    cloud_msg,  # sensor_msgs/PointCloud2 — typed loosely to avoid ROS import
    R: np.ndarray,
    translation: np.ndarray,
    z_min: float = 0.1,
    z_max: float = 2.0,
    frontal_cone_deg: float = 15.0,
) -> float | None:
    """
    Read a PointCloud2, transform it to base_footprint, apply height and
    frontal-cone filters, then return the median forward depth.

    Parameters
    ----------
    cloud_msg        : raw PointCloud2 from the depth camera
    R                : 3×3 rotation matrix (camera → base_footprint)
    translation      : (3,) translation vector (camera → base_footprint)
    z_min / z_max    : height filter bounds in base_footprint frame (m)
    frontal_cone_deg : half-angle of frontal cone (degrees)

    Returns
    -------
    median_depth : float — median XY distance in base_footprint frame, or
                   None  — if not enough points are available
    """
    if _read_points is None:
        raise RuntimeError("sensor_msgs_py is not available")  # pragma: no cover

    try:
        pts = _read_points(cloud_msg)
    except Exception:
        return None

    return _compute_rgbd_median_depth_from_array(
        _unpack_pts(pts), R, translation, z_min, z_max, frontal_cone_deg
    )


def _unpack_pts(pts) -> np.ndarray | None:
    """Convert a structured or plain array to float32 (N, 3)."""
    if pts is None or len(pts) == 0:
        return None
    if pts.dtype.names is not None:
        pts = np.column_stack([pts["x"], pts["y"], pts["z"]])
    return pts.astype(np.float32)


def _compute_rgbd_median_depth_from_array(
    pts: np.ndarray | None,
    R: np.ndarray,
    translation: np.ndarray,
    z_min: float,
    z_max: float,
    frontal_cone_deg: float,
) -> float | None:
    """
    Core depth extraction that operates on a plain (N, 3) float32 array.
    Separated from the ROS layer for unit-testing.
    """
    if pts is None or len(pts) == 0:
        return None

    pts = pts[np.isfinite(pts).all(axis=1)]
    if len(pts) == 0:
        return None

    pts = (R @ pts.T).T + translation

    pts = pts[(pts[:, 2] >= z_min) & (pts[:, 2] <= z_max)]
    if len(pts) == 0:
        return None

    cone_rad = np.deg2rad(frontal_cone_deg)
    angles_xy = np.arctan2(pts[:, 1], pts[:, 0])
    pts = pts[np.abs(angles_xy) <= cone_rad]
    if len(pts) < 5:
        return None

    forward_depth = np.linalg.norm(pts[:, :2], axis=1)
    return float(np.median(forward_depth))


# ════════════════════════════════════════════════════════════════════════════
#  CALIBRATION ROUTINE
# ════════════════════════════════════════════════════════════════════════════


def run_depth_bias_calibration(
    lidar_msg,   # sensor_msgs/LaserScan
    cloud_msg,   # sensor_msgs/PointCloud2
    R: np.ndarray,
    translation: np.ndarray,
    frontal_half_angle_deg: float = 20.0,
    min_height: float = 0.1,
    max_height: float = 2.0,
) -> float | None:
    """
    Full calibration routine — *Algorithm 1* from Delhibabu et al. (2026).

    1. Extract front-facing LiDAR points → RANSAC wall distance (z_gt).
    2. Compute median RGBD depth to the same wall (z_meas).
    3. Return Δz = z_gt − z_meas.

    Parameters
    ----------
    lidar_msg             : raw LaserScan
    cloud_msg             : raw PointCloud2
    R                     : rotation matrix (camera → base_footprint)
    translation           : translation vector (camera → base_footprint)
    frontal_half_angle_deg: frontal LiDAR arc half-width (degrees)
    min_height / max_height: height band for RGBD wall-point extraction (m)

    Returns
    -------
    delta_z : float — bias offset to add to every RGBD Z reading, or
              None  — calibration failed (not enough wall points, etc.)
    """
    ranges = np.array(lidar_msg.ranges, dtype=np.float32)
    wall_points = extract_lidar_wall_points(
        ranges=ranges,
        angle_min=lidar_msg.angle_min,
        angle_increment=lidar_msg.angle_increment,
        frontal_half_angle_deg=frontal_half_angle_deg,
        range_min=lidar_msg.range_min,
        range_max=lidar_msg.range_max,
    )
    if wall_points is None:
        return None

    z_gt = ransac_wall_distance(wall_points)
    if z_gt is None:
        return None

    z_meas = compute_rgbd_median_depth(
        cloud_msg=cloud_msg,
        R=R,
        translation=translation,
        z_min=min_height,
        z_max=max_height,
    )
    if z_meas is None:
        return None

    return z_gt - z_meas


# ════════════════════════════════════════════════════════════════════════════
#  BIAS CORRECTION & FUSION
# ════════════════════════════════════════════════════════════════════════════


def apply_depth_bias(
    pts: np.ndarray,
    delta_z: float,
    axis: int = 2,
) -> np.ndarray:
    """
    Shift one axis of a point array by *delta_z* **in-place**.

    Parameters
    ----------
    pts     : np.ndarray, shape (N, 3)
    delta_z : bias correction in metres
    axis    : column index to shift (default 2 = Z)

    Returns
    -------
    Same array with the selected axis shifted.
    Pass ``pts.copy()`` if the original must be preserved.
    """
    pts[:, axis] += delta_z
    return pts


def range_adaptive_fusion(
    lidar_bins: np.ndarray,
    rgbd_bins: np.ndarray,
    range_max: float,
    range_min: float,
    sigma_rgbd: float = 1.5,
    alpha: float = 1.0,
    rgbd_trust_max_range: float = 2.5,
) -> np.ndarray:
    """
    Fuse LiDAR and RGBD bin arrays with range-adaptive confidence weights.

    RGBD confidence decays exponentially because depth noise grows as
    σ ∝ r²  (Equation 5, Delhibabu et al. 2026):

        β(r) = exp(−r² / 2σ²)

    Fusion rules
    ────────────
    LiDAR only   → trust the LiDAR reading directly.
    RGBD only    → accept only when r < 3.0 m, else mark clear.
    Both valid   → weighted average ``(α·r_L + β·r_R) / (α + β)``,
                   clamped to ``[range_min, range_max]``.
    Neither      → range_max (clear to horizon).

    Parameters
    ----------
    lidar_bins  : (num_bins,) float32 — ``inf`` means no return
    rgbd_bins   : (num_bins,) float32 — ``inf`` means no return
    range_max   : LaserScan.range_max
    range_min   : LaserScan.range_min
    sigma_rgbd  : RGBD range-decay constant (tune for your D435i)
    alpha       : LiDAR confidence weight (constant)

    Returns
    -------
    fused : (num_bins,) float32, all values in [range_min, range_max]
    """
    fused = np.full_like(lidar_bins, range_max, dtype=np.float32)

    lidar_valid = np.isfinite(lidar_bins)
    rgbd_valid  = np.isfinite(rgbd_bins)
    both_valid  = lidar_valid & rgbd_valid

    # LiDAR only — full trust
    lidar_only = lidar_valid & ~rgbd_valid
    fused[lidar_only] = lidar_bins[lidar_only]

    # RGBD only — discard beyond 3 m (uncalibrated error too large)
    rgbd_only = rgbd_valid & ~lidar_valid
    r_rgbd_only = rgbd_bins[rgbd_only]
    fused[rgbd_only] = np.where(r_rgbd_only < rgbd_trust_max_range, r_rgbd_only, range_max)

    # Both valid — range-adaptive weighted average
    r_l = lidar_bins[both_valid]
    r_r = rgbd_bins[both_valid]
    beta  = np.exp(-(r_r ** 2) / (2.0 * sigma_rgbd ** 2))
    w_sum = alpha + beta
    fused[both_valid] = np.clip(
        (alpha * r_l + beta * r_r) / w_sum,
        range_min,
        range_max,
    )

    return fused