import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, LaserScan
from message_filters import ApproximateTimeSynchronizer, Subscriber
import numpy as np
from tf2_ros import Buffer, TransformListener
from sensor_msgs_py import point_cloud2


class DepthFusionNode2D(Node):
    def __init__(self):
        super().__init__('depth_fusion_2d')

        # LaserScan output parameters — must match your lidar's spec
        self.declare_parameter('angle_min',   -3.14159)
        self.declare_parameter('angle_max',    3.14159)
        self.declare_parameter('angle_increment', 0.00581)  # ~0.333 deg
        self.declare_parameter('range_min',    0.1)
        self.declare_parameter('range_max',   12.0)

        # Only project RGB-D points within this height band (meters)
        # Catches obstacles the lidar scan plane misses
        self.declare_parameter('min_height',  0.1)   # ignore floor
        self.declare_parameter('max_height',  2.0)   # ignore ceiling

        self.angle_min       = self.get_parameter('angle_min').value
        self.angle_max       = self.get_parameter('angle_max').value
        self.angle_increment = self.get_parameter('angle_increment').value
        self.range_min       = self.get_parameter('range_min').value
        self.range_max       = self.get_parameter('range_max').value
        self.min_height      = self.get_parameter('min_height').value
        self.max_height      = self.get_parameter('max_height').value

        self.num_bins = int(
            (self.angle_max - self.angle_min) / self.angle_increment
        )

        # Subscribers
        self.rgbd_sub  = Subscriber(self, PointCloud2, '/realsense/points')
        self.lidar_sub = Subscriber(self, LaserScan,   '/scan')

        self.sync = ApproximateTimeSynchronizer(
            [self.rgbd_sub, self.lidar_sub],
            queue_size=10,
            slop=0.05
        )
        self.sync.registerCallback(self.fusion_callback)

        self.pub = self.create_publisher(LaserScan, '/fused_scan', 10)

        self.tf_buffer   = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.target_frame = 'base_footprint'

        self.get_logger().info('DepthFusionNode2D started')

    # ── project RGB-D cloud → per-bin range array ────────────────────────────

    def rgbd_to_range_bins(self, cloud_msg):
        """
        Project an RGB-D PointCloud2 into the same angular grid as the lidar.
        Returns a float32 array of shape (num_bins,) with the closest range
        seen in each angle bin.  Bins with no point stay at inf.
        """
        bins = np.full(self.num_bins, np.inf, dtype=np.float32)

        try:
            transform = self.tf_buffer.lookup_transform(
                self.target_frame,
                cloud_msg.header.frame_id,
                cloud_msg.header.stamp,
                timeout=rclpy.duration.Duration(seconds=0.1)
            )
        except Exception as e:
            self.get_logger().warn(f'TF lookup failed: {e}')
            return bins

        try:
            pts = point_cloud2.read_points_numpy(
                cloud_msg, field_names=['x', 'y', 'z'], skip_nans=True
            )
            # convert structured array to regular Nx3 float32
            if pts.dtype.names is not None:
                pts = np.column_stack([pts['x'], pts['y'], pts['z']]).astype(np.float32)
            else:
                pts = pts.astype(np.float32)

            # Then filter NaN/Inf
            pts = pts[np.isfinite(pts).all(axis=1)]
            if pts.size == 0:
                return bins

            # ── transform to base_footprint ──────────────────────────────────
            t = transform.transform.translation
            r = transform.transform.rotation
            q = [r.x, r.y, r.z, r.w]
            if not all(np.isfinite([t.x, t.y, t.z, r.x, r.y, r.z, r.w])):
                self.get_logger().warn('TF transform contains NaN/Inf, skipping frame')
                return bins
            translation = np.array([t.x, t.y, t.z], dtype=np.float32)
            R = self._quat_to_rot(q)
            pts = (R @ pts.T).T + translation

            # ── height filter — only keep points in the obstacle band ────────
            mask = (pts[:, 2] >= self.min_height) & (pts[:, 2] <= self.max_height)
            pts = pts[mask]
            if pts.size == 0:
                return bins

            # ── project to 2D: compute angle and XY range ────────────────────
            angles = np.arctan2(pts[:, 1], pts[:, 0])   # atan2(y, x)
            ranges = np.linalg.norm(pts[:, :2], axis=1)  # XY distance only

            # ── discard out-of-range points ──────────────────────────────────
            valid = (
                (angles >= self.angle_min) & (angles <= self.angle_max) &
                (ranges >= self.range_min) & (ranges <= self.range_max)
            )
            angles = angles[valid]
            ranges = ranges[valid]

            # ── bin assignment ────────────────────────────────────────────────
            bin_indices = ((angles - self.angle_min) / self.angle_increment
                           ).astype(int)
            bin_indices = np.clip(bin_indices, 0, self.num_bins - 1)

            # Vectorized min-per-bin: for each bin keep the closest point
            np.minimum.at(bins, bin_indices, ranges)

        except Exception as e:
            self.get_logger().error(f'RGBD projection failed: {e}')

        return bins

    # ── lidar scan → per-bin range array ────────────────────────────────────

    def lidar_to_range_bins(self, scan_msg):
        """
        Re-bin the raw LaserScan onto our output angular grid.
        Handles lidars whose angle_min/increment differ from our output grid.
        """
        bins = np.full(self.num_bins, np.inf, dtype=np.float32)
 
        raw = np.array(scan_msg.ranges, dtype=np.float32)

        # Angles for each raw beam
        # raw_angles = (scan_msg.angle_min +
        #               np.arange(len(raw)) * scan_msg.angle_increment)
        raw_angles = np.where(raw_angles > np.pi,
                          raw_angles - 2 * np.pi,
                          raw_angles)

        # Filter valid range values
        valid = (
            np.isfinite(raw) &
            (raw >= scan_msg.range_min) &
            (raw <= scan_msg.range_max) &
            (raw_angles >= self.angle_min) &
            (raw_angles <= self.angle_max)
        )
        raw_angles = raw_angles[valid]
        raw        = raw[valid]

        bin_indices = ((raw_angles - self.angle_min) / self.angle_increment
                       ).astype(int)
        bin_indices = np.clip(bin_indices, 0, self.num_bins - 1)

        np.minimum.at(bins, bin_indices, raw)
        return bins

    # ── fusion callback ──────────────────────────────────────────────────────

    def fusion_callback(self, rgbd_msg, lidar_msg):

        rgbd_bins  = self.rgbd_to_range_bins(rgbd_msg)
        lidar_bins = self.lidar_to_range_bins(lidar_msg)

        # ── fusion strategy: trust lidar where it has data, ──────────────────
        # fill gaps with RGB-D, keep min where both see something
        fused = np.full(self.num_bins, np.inf, dtype=np.float32)

        lidar_valid = np.isfinite(lidar_bins)
        rgbd_valid  = np.isfinite(rgbd_bins)
        both_valid  = lidar_valid & rgbd_valid

        # Lidar only
        fused[lidar_valid & ~rgbd_valid] = lidar_bins[lidar_valid & ~rgbd_valid]

        # RGB-D only (fills blind spots)
        fused[rgbd_valid & ~lidar_valid] = rgbd_bins[rgbd_valid & ~lidar_valid]

        # Both: take the closer (more conservative / safer) reading
        fused[both_valid] = np.minimum(
            lidar_bins[both_valid],
            rgbd_bins[both_valid]
        )

        # Replace inf with 0.0 — LaserScan convention for "no return"
        fused[~np.isfinite(fused)] = 0.0

        # ── build output LaserScan ────────────────────────────────────────────
        out = LaserScan()
        out.header.stamp    = lidar_msg.header.stamp
        out.header.frame_id = self.target_frame
        out.angle_min       = self.angle_min
        out.angle_max       = self.angle_max
        out.angle_increment = self.angle_increment
        out.time_increment  = 0.0
        out.scan_time       = lidar_msg.scan_time
        out.range_min       = self.range_min
        out.range_max       = self.range_max
        out.ranges          = fused.tolist()
        out.intensities     = []

        self.pub.publish(out)

    # ── helper ───────────────────────────────────────────────────────────────

    def _quat_to_rot(self, q):
        x, y, z, w = q
        return np.array([
            [1-2*(y*y+z*z),   2*(x*y-z*w),   2*(x*z+y*w)],
            [  2*(x*y+z*w), 1-2*(x*x+z*z),   2*(y*z-x*w)],
            [  2*(x*z-y*w),   2*(y*z+x*w), 1-2*(x*x+y*y)]
        ], dtype=np.float32)


def main(args=None):
    rclpy.init(args=args)
    node = DepthFusionNode2D()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()