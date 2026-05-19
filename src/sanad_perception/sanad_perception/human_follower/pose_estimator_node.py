"""TrackedPersonArray + CameraInfo → PoseStamped in the map frame.

Back-projects the target person's centroid pixel + depth into a 3D point
using the pinhole camera model, then transforms it to the map frame via TF2.

Subscribed
----------
  /person/tracks          person_following_robot/TrackedPersonArray
  /camera/color/camera_info  sensor_msgs/CameraInfo  (latched, once)

Published
---------
  /person/pose            geometry_msgs/PoseStamped  (map frame)

Target selection
----------------
  • If TrackedPersonArray.target_id >= 0, that track is followed.
  • Otherwise the closest person with a valid depth reading is chosen
    and its track_id is latched until it disappears for > lost_timeout sec.
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time

import tf2_ros
import tf2_geometry_msgs  # registers PoseStamped transform support

from geometry_msgs.msg import PoseStamped, PointStamped
from sensor_msgs.msg import CameraInfo
from sanad_interfaces.msg import TrackedPersonArray


_DEFAULTS = {
    'camera_frame': 'depth_camera_link',
    'map_frame':    'map',
    'tf_timeout':   0.1,    # seconds to wait for a TF transform
    'lost_timeout': 2.0,    # seconds before dropping a latched target_id
}


class PoseEstimatorNode(Node):

    def __init__(self):
        super().__init__('pose_estimator_node')

        for name, default in _DEFAULTS.items():
            self.declare_parameter(name, default)

        def p(name):
            return self.get_parameter(name).value

        self._camera_frame = p('camera_frame')
        self._map_frame    = p('map_frame')
        self._tf_timeout   = Duration(seconds=p('tf_timeout'))
        self._lost_timeout = p('lost_timeout')

        self._tf_buffer    = tf2_ros.Buffer()
        self._tf_listener  = tf2_ros.TransformListener(self._tf_buffer, self)

        # Camera intrinsics — populated once from /camera_info
        self._fx = self._fy = self._cx = self._cy = None

        # Latched target state
        self._target_id:      int   = -1
        self._last_seen_time: float = 0.0   # rclpy.time in seconds

        self.create_subscription(CameraInfo,        '/realsense/camera_info', self._on_camera_info, 10)
        self.create_subscription(TrackedPersonArray, '/person/tracks',            self._on_tracks,      10)

        self._pose_pub = self.create_publisher(PoseStamped, '/person/pose', 10)

        self.get_logger().info('PoseEstimatorNode ready')

    # ── camera info ──────────────────────────────────────────────────────────

    def _on_camera_info(self, msg: CameraInfo):
        if self._fx is not None:
            return  # already have intrinsics
        self._fx = msg.k[0]
        self._fy = msg.k[4]
        self._cx = msg.k[2]
        self._cy = msg.k[5]
        self.get_logger().info(
            f'Intrinsics: fx={self._fx:.1f} fy={self._fy:.1f} '
            f'cx={self._cx:.1f} cy={self._cy:.1f}'
        )

    # ── main callback ────────────────────────────────────────────────────────

    def _on_tracks(self, msg: TrackedPersonArray):
        if self._fx is None:
            self.get_logger().warn('Waiting for camera_info…', throttle_duration_sec=5.0)
            return

        target = self._select_target(msg)
        if target is None:
            return

        if not target.depth_valid:
            return

        pose = self._to_map_pose(target, msg.header.stamp)
        if pose is not None:
            self._pose_pub.publish(pose)

    # ── target selection ─────────────────────────────────────────────────────

    def _select_target(self, msg: TrackedPersonArray):
        """Return the TrackedPerson to follow, or None."""
        now = self.get_clock().now().nanoseconds * 1e-9

        # External node has designated a specific target
        if msg.target_id >= 0:
            track = self._find_by_id(msg.tracks, msg.target_id)
            if track is not None:
                self._target_id    = msg.target_id
                self._last_seen_time = now
            return track

        # We have a latched target from a previous frame
        if self._target_id >= 0:
            track = self._find_by_id(msg.tracks, self._target_id)
            if track is not None:
                self._last_seen_time = now
                return track
            # Target gone — wait out the timeout before resetting
            if (now - self._last_seen_time) < self._lost_timeout:
                return None
            self._target_id = -1

        # No target yet — pick the closest person with a valid depth
        valid = [t for t in msg.tracks if t.depth_valid]
        if not valid:
            return None

        target = min(valid, key=lambda t: t.depth_m)
        self._target_id    = target.track_id
        self._last_seen_time = now
        self.get_logger().info(f'Acquired target track_id={self._target_id}')
        return target

    @staticmethod
    def _find_by_id(tracks, track_id: int):
        return next((t for t in tracks if t.track_id == track_id), None)

    # ── back-projection + TF ─────────────────────────────────────────────────

    def _to_map_pose(self, track, stamp) -> PoseStamped | None:
        """
        Pinhole back-projection → camera frame point → map frame PoseStamped.

        Camera optical frame convention: Z forward, X right, Y down.
        """
        z = float(track.depth_m)
        x = (float(track.centroid_x) - self._cx) * z / self._fx
        y = (float(track.centroid_y) - self._cy) * z / self._fy

        point_cam = PointStamped()
        point_cam.header.frame_id = self._camera_frame
        point_cam.header.stamp    = stamp
        point_cam.point.x = x
        point_cam.point.y = y
        point_cam.point.z = z

        try:
            point_map = self._tf_buffer.transform(
                point_cam, self._map_frame, timeout=self._tf_timeout
            )
        except (tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as e:
            self.get_logger().warn(f'TF error: {e}', throttle_duration_sec=2.0)
            return None

        pose = PoseStamped()
        pose.header.frame_id = self._map_frame
        pose.header.stamp    = stamp
        pose.pose.position.x = point_map.point.x
        pose.pose.position.y = point_map.point.y
        pose.pose.position.z = 0.0          # flatten to ground plane for Nav2
        pose.pose.orientation.w = 1.0       # orientation set by the goal node
        return pose


# ── entry point ──────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = PoseEstimatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()