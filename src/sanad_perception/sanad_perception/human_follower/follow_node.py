#!/usr/bin/env python3
"""
TrackedPerson → back-project to map frame → NavigateToPose action.

This node merges two previously separate concerns:

  1. POSE ESTIMATION  (§ _to_map_pose)
     Pinhole back-projection of the target's centroid pixel + depth into
     a 3D point in the camera optical frame, then TF2 transform to map.

  2. GOAL MANAGEMENT  (§ _on_target onwards)
     Rate-limited, displacement-gated Nav2 goal sending with a follow-
     distance offset and a watchdog that cancels when the person is lost.

Subscribed
----------
  /person/target          sanad_interfaces/TrackedPerson
  /realsense/camera_info  sensor_msgs/CameraInfo  (read once)

Action client
-------------
  navigate_to_pose        nav2_msgs/action/NavigateToPose

Debug publisher (optional)
--------------------------
  /person/pose            geometry_msgs/PoseStamped  (map frame)
"""

import math

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time

import tf2_ros
import tf2_geometry_msgs

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, PointStamped, Quaternion
from sensor_msgs.msg import CameraInfo
from nav2_msgs.action import NavigateToPose
from sanad_interfaces.msg import TrackedPerson

# Terminal statuses in ROS2 Humble — ClientGoalHandle has no .done() method
_TERMINAL = {GoalStatus.STATUS_SUCCEEDED,
             GoalStatus.STATUS_CANCELED,
             GoalStatus.STATUS_ABORTED}


_DEFAULTS = {
    # ── pose estimation ──────────────────────────────────────────────────────
    'camera_frame':       'depth_camera_link',
    'map_frame':          'map',
    'tf_timeout':          0.1,

    # ── goal management ──────────────────────────────────────────────────────
    'follow_distance_m':   1.0,    # stop this far behind the person
    'min_person_disp_m':   0.20,   # min person movement to trigger a new goal
    'min_goal_disp_m':     0.40,   # min goal displacement before cancelling active goal
    'goal_update_hz':      1.0,    # max rate at which new goals are sent
    'lost_timeout':        2.0,    # seconds without a pose before cancelling
    'base_frame':         'base_link',

    # ── debug ────────────────────────────────────────────────────────────────
    'publish_pose':        False,  # publish /person/pose for external monitoring
}


def _yaw_to_quat(yaw: float) -> Quaternion:
    q = Quaternion()
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


def _dist2d(a, b) -> float:
    return math.hypot(a.x - b.x, a.y - b.y)


class FollowNode(Node):

    def __init__(self):
        super().__init__('follow_node')

        for name, default in _DEFAULTS.items():
            self.declare_parameter(name, default)

        def p(name):
            return self.get_parameter(name).value
        

        # ── pose estimation params ───────────────────────────────────────────
        self._camera_frame = p('camera_frame')
        self._map_frame    = p('map_frame')
        self._tf_timeout   = Duration(seconds=p('tf_timeout'))

        self._tf_buffer   = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        self._fx = self._fy = self._cx = self._cy = None   # populated on first CameraInfo

        # ── goal management params ───────────────────────────────────────────
        self._follow_dist     = p('follow_distance_m')
        self._min_person_disp = p('min_person_disp_m')
        self._min_goal_disp   = p('min_goal_disp_m')
        self._goal_period     = 1.0 / p('goal_update_hz')
        self._lost_timeout    = p('lost_timeout')
        self._base_frame      = p('base_frame')

        self._goal_handle        = None
        self._last_person_pos    = None
        self._last_sent_goal_pos = None
        self._last_goal_time     = 0.0


        # ── subscriptions ────────────────────────────────────────────────────
        self.create_subscription(CameraInfo,    '/realsense/camera_info', self._on_camera_info, 10)
        self.create_subscription(TrackedPerson, '/person/target',         self._on_target,      10)

        # ── action client ────────────────────────────────────────────────────
        self._action = ActionClient(self, NavigateToPose, 'navigate_to_pose')


        # ── optional debug publisher ─────────────────────────────────────────
        self._pose_pub = (
            self.create_publisher(PoseStamped, '/person/pose', 10)
            if p('publish_pose') else None
        )

        # Watchdog: fires when person disappears
        self._watchdog = self.create_timer(self._lost_timeout, self._on_lost)

        self.get_logger().info('FollowNode ready — waiting for Nav2 action server…')
        self._action.wait_for_server()
        self.get_logger().info('Nav2 action server connected')

    # =========================================================================
    # § POSE ESTIMATION
    # =========================================================================

    def _on_camera_info(self, msg: CameraInfo):
        if self._fx is not None:
            return
        self._fx, self._fy = msg.k[0], msg.k[4]
        self._cx, self._cy = msg.k[2], msg.k[5]
        self.get_logger().info(
            f'Intrinsics: fx={self._fx:.1f} fy={self._fy:.1f} '
            f'cx={self._cx:.1f} cy={self._cy:.1f}'
        )

    def _to_map_pose(self, track: TrackedPerson) -> PoseStamped | None:
        """
        Pinhole back-projection → camera frame point → map frame PoseStamped.
        Camera optical convention: Z forward, X right, Y down.
        """
        z = float(track.depth_m)
        x = (float(track.centroid_x) - self._cx) * z / self._fx
        y = (float(track.centroid_y) - self._cy) * z / self._fy

        point_cam = PointStamped()
        point_cam.header.frame_id = self._camera_frame
        point_cam.header.stamp    = track.header.stamp
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
        pose.header.stamp    = track.header.stamp
        pose.pose.position.x = point_map.point.x
        pose.pose.position.y = point_map.point.y
        pose.pose.position.z = 0.0       # flattened to ground plane for Nav2
        pose.pose.orientation.w = 1.0    # orientation set by _compute_goal
        return pose
    
    # =========================================================================
    # § GOAL MANAGEMENT
    # =========================================================================

    def _on_target(self, track: TrackedPerson):
        self._watchdog.reset()

        if self._fx is None:
            self.get_logger().warn('Waiting for camera_info…', throttle_duration_sec=5.0)
            return

        if not track.depth_valid:
            return

        pose = self._to_map_pose(track)
        if pose is None:
            return

        if self._pose_pub:
            self._pose_pub.publish(pose)

        person_pos = pose.pose.position

        if not self._should_update(person_pos):
            return

        robot_pos = self._robot_position()
        if robot_pos is None:
            return

        goal_pose = self._compute_goal(person_pos, robot_pos)
        if goal_pose is None:
            return

        if (self._last_sent_goal_pos is not None and
                _dist2d(goal_pose.pose.position, self._last_sent_goal_pos)
                < self._min_goal_disp):
            self._last_person_pos = person_pos
            self._last_goal_time  = self.get_clock().now().nanoseconds * 1e-9
            return

        self._send_goal(goal_pose, person_pos)

    def _should_update(self, person_pos) -> bool:
        now = self.get_clock().now().nanoseconds * 1e-9
        if (now - self._last_goal_time) < self._goal_period:
            return False
        if (self._last_person_pos is not None and
                _dist2d(person_pos, self._last_person_pos) < self._min_person_disp):
            return False
        return True

    def _robot_position(self):
        try:
            tf = self._tf_buffer.lookup_transform(
                self._map_frame, self._base_frame,
                Time(),
                timeout=self._tf_timeout,
            )
            return tf.transform.translation
        except tf2_ros.TransformException as e:
            self.get_logger().warn(f'TF robot position: {e}', throttle_duration_sec=5.0)
            return None

    def _compute_goal(self, person_pos, robot_pos) -> PoseStamped | None:
        dx   = person_pos.x - robot_pos.x
        dy   = person_pos.y - robot_pos.y
        dist = math.hypot(dx, dy)

        if dist < self._follow_dist:
            return None

        scale = (dist - self._follow_dist) / dist

        pose = PoseStamped()
        pose.header.frame_id  = self._map_frame
        pose.header.stamp     = self.get_clock().now().to_msg()
        pose.pose.position.x  = robot_pos.x + dx * scale
        pose.pose.position.y  = robot_pos.y + dy * scale
        pose.pose.position.z  = 0.0
        pose.pose.orientation = _yaw_to_quat(math.atan2(dy, dx))
        return pose

    def _goal_active(self) -> bool:
        return (self._goal_handle is not None and
                self._goal_handle.status not in _TERMINAL)

    def _send_goal(self, pose: PoseStamped, person_pos=None):
        if self._goal_active():
            self._goal_handle.cancel_goal()

        future = self._action.send_goal_async(NavigateToPose.Goal(pose=pose))
        future.add_done_callback(self._on_goal_accepted)

        self._last_sent_goal_pos = pose.pose.position
        self._last_person_pos    = person_pos
        self._last_goal_time     = self.get_clock().now().nanoseconds * 1e-9

    def _on_goal_accepted(self, future):
        self._goal_handle = future.result()
        if not self._goal_handle.accepted:
            self.get_logger().warn('Goal rejected by Nav2')
            return
        self._goal_handle.get_result_async().add_done_callback(self._on_goal_result)

    def _on_goal_result(self, future):
        status = future.result().status
        if status not in (GoalStatus.STATUS_SUCCEEDED, GoalStatus.STATUS_CANCELED):
            self.get_logger().warn(f'Goal ended with status {status}')

    # ── watchdog ─────────────────────────────────────────────────────────────

    def _on_lost(self):
        """Person disappeared — cancel the active Nav2 goal."""
        if self._goal_active():
            self.get_logger().info('Person lost — cancelling goal')
            self._goal_handle.cancel_goal()
        self._last_person_pos    = None
        self._last_sent_goal_pos = None


# ── entry point ──────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = FollowNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()