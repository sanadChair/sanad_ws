# depth_fusion_node.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, LaserScan          # CHANGED: LaserScan replaces PointCloud2 for lidar
from message_filters import ApproximateTimeSynchronizer, Subscriber
import numpy as np
from tf2_ros import Buffer, TransformListener
from sensor_msgs_py import point_cloud2
import tf2_sensor_msgs
from laser_geometry import LaserProjection                  # NEW: converts LaserScan → PointCloud2
import warnings
# Suppress the harmless NaN warning from laser_geometry's internal math
warnings.filterwarnings("ignore", message=".*invalid value encountered.*")

class DepthFusionNode3D(Node):
    def __init__(self):
        super().__init__('depth_fusion_3d')

        # NEW: projects polar LaserScan beams into a flat cartesian PointCloud2.
        # Each beam becomes an (x, y, z=0) point in the lidar's own frame.
        self.laser_projector = LaserProjection()
        
        # Subscribers with message_filters for sync
        self.rgbd_sub = Subscriber(self, PointCloud2, '/realsense/points')
        self.lidar_sub = Subscriber(self, LaserScan, '/scan')
        
        # CHANGED: slop increased from 10ms → 50ms.
        # 2D lidars typically publish at ~10 Hz vs RGB-D at 30 Hz,
        # so a tighter tolerance would drop most frame pairs.
        self.sync = ApproximateTimeSynchronizer(
            [self.rgbd_sub, self.lidar_sub],
            queue_size=10,
            slop=0.05
        )
        self.sync.registerCallback(self.fusion_callback)
        
        self.pub = self.create_publisher(PointCloud2, '/fused/points', 10)

        # TF2 for coordinate transforms
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.target_frame = 'base_footprint'

    # ── transform helper (unchanged) ────────────────────────────────────────
 
    def quaternion_to_rotation_matrix(self, q):
        """Convert quaternion [x, y, z, w] to 3x3 rotation matrix."""
        x, y, z, w = q
        return np.array([
            [1 - 2*(y*y + z*z),     2*(x*y - z*w),     2*(x*z + y*w)],
            [2*(x*y + z*w),         1 - 2*(x*x + z*z), 2*(y*z - x*w)],
            [2*(x*z - y*w),         2*(y*z + x*w),     1 - 2*(x*x + y*y)]
        ])

    def transform_cloud(self, cloud_msg, target_frame):
        """
        Manually transform point cloud using TF2 and numpy.
        This avoids tf2_sensor_msgs strict field-matching requirements.
        """
        try:
            transform = self.tf_buffer.lookup_transform(
                target_frame,
                cloud_msg.header.frame_id,
                cloud_msg.header.stamp,
                timeout=rclpy.duration.Duration(seconds=0.1)
            )
        except Exception as e:
            self.get_logger().error(f'Transform lookup failed: {e}')
            return None
        
        try:
            # Read XYZ points as numpy array
            points_gen = point_cloud2.read_points(
                cloud_msg,
                field_names=['x', 'y', 'z'],
                skip_nans=True
            )
            
            # # Convert generator output to a clean numpy array
            points = np.array(list(points_gen), dtype=np.float32)
            
            if points.size == 0:
                return None
                
            # Filter invalid points (NaN/Inf)
            valid_mask = np.isfinite(points).all(axis=1)
            if not valid_mask.any():
                return None
            
            # valid_points = points[valid_mask]

            points = np.array(list(points_gen))
            if points.size == 0:
                return None
            
            # CHANGED: Correctly convert structured array to a regular Nx3 float32 array
            if points.dtype.names is not None:
                points = np.column_stack([points['x'], points['y'], points['z']]).astype(np.float32)
            
            # Extract transform components
            t = transform.transform.translation
            r = transform.transform.rotation
            q = [r.x, r.y, r.z, r.w]
            translation = np.array([t.x, t.y, t.z])
            R = self.quaternion_to_rotation_matrix(q)
            
            # Apply rigid transform: p' = R @ p + t
            transformed = (R @ points.T).T + translation
            
            # Create new PointCloud2 message
            new_msg = point_cloud2.create_cloud_xyz32(
                cloud_msg.header,
                transformed
            )
            new_msg.header.frame_id = target_frame
            new_msg.header.stamp = cloud_msg.header.stamp
            
            return new_msg
            
        except Exception as e:
            self.get_logger().error(f'Point transform failed: {e}')
            return None
        
     # ── main callback ────────────────────────────────────────────────────────
        
    def fusion_callback(self, rgbd_msg, lidar_msg):

        # NEW: convert 2D LaserScan → PointCloud2 before any further processing.
        # projectLaser() handles polar-to-cartesian conversion and sets z = 0 for
        # every beam. Out-of-range beams are projected as NaN — filtered below.
        try:
            lidar_cloud_msg = self.laser_projector.projectLaser(lidar_msg)
        except Exception as e:
            self.get_logger().error(f'LaserScan projection failed: {e}')
            return
        
        # Transform both to base_footprint
        rgbd_cloud = self.transform_cloud(rgbd_msg, self.target_frame)
        lidar_cloud = self.transform_cloud(lidar_cloud_msg, self.target_frame)
        
        if rgbd_cloud is None or lidar_cloud is None:
            return
        
        # Convert to numpy arrays for processing
        # Read RGB-D points — drop color channels, keep xyz only for fusion
        rgbd_points = point_cloud2.read_points_numpy(
            rgbd_cloud,
            field_names=['x', 'y', 'z']
        )

        # CHANGED: projected LaserScan cloud only has x, y, z (and possibly
        # 'intensity'). Do NOT request 'rgb' — it doesn't exist here.
        lidar_points = point_cloud2.read_points_numpy(
            lidar_cloud,
            field_names=['x', 'y', 'z']
        )

        # NEW: filter NaN / Inf.
        # projectLaser() writes NaN for every beam that fell outside
        # [range_min, range_max]. RGB-D cameras also produce NaN for
        # pixels with no depth return. Both must be removed before any
        # distance math, or numpy silently propagates NaN through everything.
        rgbd_points  = rgbd_points [np.isfinite(rgbd_points ).all(axis=1)]
        lidar_points = lidar_points[np.isfinite(lidar_points).all(axis=1)]
 
        if len(rgbd_points) == 0 or len(lidar_points) == 0:
            return
        
        # CHANGED: use 2D (XY) distance for both sensors.
        # Lidar z is always ≈ 0, so mixing it into a 3D norm would skew
        # the result when comparing against elevated RGB-D points.
        rgbd_dist  = np.linalg.norm(rgbd_points [:, :2], axis=1)
        lidar_dist = np.linalg.norm(lidar_points[:, :2], axis=1)
        
        # Weight functions — tuned for 2D lidar range ceiling (~12m):
        #   RGB-D  : reliable up to ~4m, decays smoothly past that
        #   LiDAR  : sigmoid centered at 3m (was 5m in original — that was for
        #            a 3D lidar with a much longer reliable range)
        rgbd_weights  = np.exp(-rgbd_dist / 3.0)
        lidar_weights = 1.0 / (1.0 + np.exp(-2.0 * (lidar_dist - 3.0)))
        
        # Fuse in overlapping regions using weighted average
        fused_points = self.weighted_fusion_2d(
            rgbd_points, lidar_points,
            rgbd_weights, lidar_weights,
            lidar_dist                              # FIX: pass lidar_dist as parameter
        )
        
        if len(fused_points) == 0:
            return
 
        fused_msg = point_cloud2.create_cloud_xyz32(
            rgbd_cloud.header,
            fused_points
        )
        self.pub.publish(fused_msg)


    # ── fusion logic ─────────────────────────────────────────────────────────

    def weighted_fusion_2d(self, rgbd, lidar, w_rgbd, w_lidar, lidar_dist):
        # Voxelize space into 10cm cubes for efficient lookup
        from scipy.spatial import cKDTree
        
        
        # Build KD-tree for fast nearest neighbor search
        # CHANGED: build the KD-tree on XY only (2 columns), not XYZ.
        # The original used 3D positions, which is correct for a 3D lidar that
        # has real z data. For a 2D lidar z ≈ 0, so a 3D query would reject any
        # RGB-D point above ground level (e.g. a wall at z = 0.5m would never
        # match a lidar point at z ≈ 0 even when they're at the same XY).
        lidar_tree = cKDTree(lidar[:, :2])
        
        fused = []
        for i, point in enumerate(rgbd):
            # Find nearest lidar point within 20cm
            # CHANGED: query by XY position only (point[:2])
            dist, idx = lidar_tree.query(point[:2], distance_upper_bound=0.2)
            
            if dist < 0.2:  # Overlap detected
                # Kalman fusion: weighted average with uncertainty
                # Overlap detected in the horizontal plane.
                w_r = w_rgbd[i]
                w_l = w_lidar[idx]
                
                # CHANGED: fuse X and Y with weighted average (same as before),
                # but keep the RGB-D Z unchanged. The lidar z is ≈ 0 regardless
                # of the real obstacle height — using a weighted average of Z
                # would drag every point toward the ground. The camera's depth
                # gives the only meaningful height information here.
                fused_xy    = (w_r * point[:2] + w_l * lidar[idx, :2]) / (w_r + w_l)
                fused_point = np.array([fused_xy[0], fused_xy[1], point[2]])
                fused.append(fused_point)
            else:
                # No overlap, use RGB-D point as-is
                fused.append(point)
        
        # Add lidar-only points beyond RGB-D reliable range.
        # CHANGED: threshold lowered from 5m → 4m because a typical Intel
        # RealSense / Astra has a reliable range of ~4m (not 5m).
        far_mask = lidar_dist > 4.0
        if far_mask.any():
            # Lidar points beyond RGB-D range have z ≈ 0. That's fine — for
            # 2D SLAM these ground-plane points are exactly what slam_toolbox
            # and AMCL need.
            fused.extend(lidar[far_mask])
        
        return np.array(fused) if fused else np.empty((0, 3))
    

def main(args=None):
    rclpy.init(args=args)
    node = DepthFusionNode3D()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
 
 
if __name__ == '__main__':
    main()


