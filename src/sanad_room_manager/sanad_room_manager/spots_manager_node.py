#!/usr/bin/env python3
import math
import os
import colorsys
import time

import yaml
from ament_index_python import get_package_share_directory
import rclpy
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA

from sanad_interfaces.msg import Spot
from sanad_interfaces.srv import AddSpot, DeleteSpot, GetAllSpots, GetSpot, NavigateToSpot


try:
    sanad_room_manager_pkg = get_package_share_directory("sanad_room_manager")
except Exception:
    sanad_room_manager_pkg = None


class SpotsManagerNode(Node):
    def __init__(self):
        super().__init__("spots_manager_node")
        self.callback_group = ReentrantCallbackGroup()
        self.spots_yaml_path = self._resolve_spots_yaml_path()
        self.spots_dict = self._load_spots_dict()

        # Action client for Nav2
        self.nav_client = ActionClient(
            self,
            NavigateToPose,
            "navigate_to_pose",
            callback_group=self.callback_group,
        )

        # RViz Marker Publishers with TRANSIENT_LOCAL QoS for RViz persistence
        marker_qos = QoSProfile(
            depth=10,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.marker_pub = self.create_publisher(
            MarkerArray, "/spots_markers", qos_profile=marker_qos
        )
        self.visualization_marker_pub = self.create_publisher(
            MarkerArray, "/visualization_marker_array", qos_profile=marker_qos
        )
        self.single_marker_pub = self.create_publisher(
            Marker, "/visualization_marker", qos_profile=marker_qos
        )

        # Services
        self.get_srv = self.create_service(
            GetSpot, "spots/get", self.handle_get_spot, callback_group=self.callback_group
        )
        self.add_srv = self.create_service(
            AddSpot, "spots/add", self.handle_add_spot, callback_group=self.callback_group
        )
        self.delete_srv = self.create_service(
            DeleteSpot, "spots/delete", self.handle_delete_spot, callback_group=self.callback_group
        )
        self.get_all_srv = self.create_service(
            GetAllSpots, "spots/get_all", self.handle_get_all_spots, callback_group=self.callback_group
        )
        self.navigate_srv = self.create_service(
            NavigateToSpot, "spots/navigate", self.handle_navigate_to_spot, callback_group=self.callback_group
        )

        # Timer for publishing visualization markers continuously (every 1.0s)
        self.timer = self.create_timer(1.0, self.publish_markers, callback_group=self.callback_group)

        self.get_logger().info("✅ Spots Manager Node started.")
        self.publish_markers()

    def _resolve_spots_yaml_path(self):
        if sanad_room_manager_pkg:
            return os.path.join(sanad_room_manager_pkg, "config", "spots.yaml")
        return "config/spots.yaml"

    def _load_spots_yaml(self):
        try:
            with open(self.spots_yaml_path, "r", encoding="utf-8") as yaml_file:
                data = yaml.safe_load(yaml_file) or {}
        except Exception as exc:
            self.get_logger().error(f"Failed to load spots YAML '{self.spots_yaml_path}': {exc}")
            return {}

        if isinstance(data, dict):
            return data

        self.get_logger().error("Invalid spots YAML format: expected a dictionary")
        return {}

    def _load_spots_dict(self):
        spots_data = self._load_spots_yaml()
        spots_dict = {}
        for title, entry in spots_data.items():
            spots_dict[str(title)] = self._build_spot_msg(str(title), entry)
        self.get_logger().info(f"Loaded {len(spots_dict)} spots into local cache.")
        return spots_dict

    def _build_spot_msg(self, title: str, entry: dict) -> Spot:
        pose = entry.get("pose", {}) if isinstance(entry, dict) else {}
        spot = Spot()
        spot.title = title
        spot.pose_x = float(pose.get("x", 0.0))
        spot.pose_y = float(pose.get("y", 0.0))
        spot.pose_z = float(pose.get("z", 0.0))
        spot.pose_theta = float(pose.get("theta", 0.0))
        return spot

    def _spot_to_yaml_entry(self, spot: Spot):
        return {
            "pose": {
                "x": float(spot.pose_x),
                "y": float(spot.pose_y),
                "z": float(spot.pose_z),
                "theta": float(spot.pose_theta),
            }
        }

    def _persist_spots_yaml(self):
        serialized_spots = {}
        for title, spot in self.spots_dict.items():
            serialized_spots[title] = self._spot_to_yaml_entry(spot)

        try:
            with open(self.spots_yaml_path, "w", encoding="utf-8") as yaml_file:
                yaml.safe_dump(
                    serialized_spots,
                    yaml_file,
                    sort_keys=False,
                    default_flow_style=False,
                )
            return True
        except Exception as exc:
            self.get_logger().error(f"Failed to persist spots YAML '{self.spots_yaml_path}': {exc}")
            return False

    def _get_spot_color(self, title: str) -> ColorRGBA:
        # Deterministically compute a vibrant random RGB color from the title hash
        h = (abs(hash(title)) % 360) / 360.0
        r, g, b = colorsys.hsv_to_rgb(h, 0.85, 0.95)
        color = ColorRGBA()
        color.r = float(r)
        color.g = float(g)
        color.b = float(b)
        color.a = 0.85
        return color

    def _find_spot_case_insensitive(self, title: str):
        target = title.strip().lower()
        for key, spot in self.spots_dict.items():
            if key.strip().lower() == target:
                return spot
        return None

    def publish_markers(self, clear_first=False):
        marker_array = MarkerArray()

        now = self.get_clock().now().to_msg()

        if clear_first:
            clear_marker = Marker()
            clear_marker.action = Marker.DELETEALL
            clear_array = MarkerArray()
            clear_array.markers.append(clear_marker)
            self.marker_pub.publish(clear_array)
            self.visualization_marker_pub.publish(clear_array)

        for idx, (title, spot) in enumerate(self.spots_dict.items()):
            color = self._get_spot_color(title)
            half_yaw = float(spot.pose_theta) * 0.5
            qx = 0.0
            qy = 0.0
            qz = math.sin(half_yaw)
            qw = math.cos(half_yaw)

            # 1. Rectangle (CUBE) Marker representing the spot shape on the floor
            rect_marker = Marker()
            rect_marker.header.frame_id = "map"
            rect_marker.header.stamp = now
            rect_marker.ns = "spot_rectangles"
            rect_marker.id = idx * 3 + 0
            rect_marker.type = Marker.CUBE
            rect_marker.action = Marker.ADD
            rect_marker.pose.position.x = float(spot.pose_x)
            rect_marker.pose.position.y = float(spot.pose_y)
            rect_marker.pose.position.z = float(spot.pose_z) + 0.025
            rect_marker.pose.orientation.x = qx
            rect_marker.pose.orientation.y = qy
            rect_marker.pose.orientation.z = qz
            rect_marker.pose.orientation.w = qw
            rect_marker.scale.x = 0.5   # Length of rectangle
            rect_marker.scale.y = 0.35  # Width of rectangle
            rect_marker.scale.z = 0.05  # Height of rectangle
            rect_marker.color = color
            rect_marker.color.a = 0.85
            marker_array.markers.append(rect_marker)

            # 2. Arrow Marker indicating direction of orientation
            arrow_marker = Marker()
            arrow_marker.header.frame_id = "map"
            arrow_marker.header.stamp = now
            arrow_marker.ns = "spot_orientations"
            arrow_marker.id = idx * 3 + 1
            arrow_marker.type = Marker.ARROW
            arrow_marker.action = Marker.ADD
            arrow_marker.pose.position.x = float(spot.pose_x)
            arrow_marker.pose.position.y = float(spot.pose_y)
            arrow_marker.pose.position.z = float(spot.pose_z) + 0.08
            arrow_marker.pose.orientation.x = qx
            arrow_marker.pose.orientation.y = qy
            arrow_marker.pose.orientation.z = qz
            arrow_marker.pose.orientation.w = qw
            arrow_marker.scale.x = 0.6  # Shaft length
            arrow_marker.scale.y = 0.1  # Shaft diameter
            arrow_marker.scale.z = 0.1  # Head diameter
            arrow_color = ColorRGBA()
            arrow_color.r = min(1.0, color.r + 0.25)
            arrow_color.g = min(1.0, color.g + 0.25)
            arrow_color.b = min(1.0, color.b + 0.25)
            arrow_color.a = 1.0
            arrow_marker.color = arrow_color
            marker_array.markers.append(arrow_marker)

            # 3. Text View Facing Marker displaying Spot Title above it
            text_marker = Marker()
            text_marker.header.frame_id = "map"
            text_marker.header.stamp = now
            text_marker.ns = "spot_labels"
            text_marker.id = idx * 3 + 2
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            text_marker.pose.position.x = float(spot.pose_x)
            text_marker.pose.position.y = float(spot.pose_y)
            text_marker.pose.position.z = float(spot.pose_z) + 0.45
            text_marker.pose.orientation.w = 1.0
            text_marker.scale.z = 0.3  # Text height
            text_marker.text = title
            text_marker.color.r = 1.0
            text_marker.color.g = 1.0
            text_marker.color.b = 1.0
            text_marker.color.a = 1.0
            marker_array.markers.append(text_marker)

        self.marker_pub.publish(marker_array)
        self.visualization_marker_pub.publish(marker_array)
        for m in marker_array.markers:
            self.single_marker_pub.publish(m)

    def handle_get_spot(self, request: GetSpot.Request, response: GetSpot.Response):
        title = request.title.strip()
        spot = self._find_spot_case_insensitive(title)
        if spot is None:
            response.success = False
            self.get_logger().warn(f"Spot not found: '{title}'")
            return response

        response.success = True
        response.spot = spot
        return response

    def handle_get_all_spots(self, request: GetAllSpots.Request, response: GetAllSpots.Response):
        del request
        response.spots = list(self.spots_dict.values())
        return response

    def handle_add_spot(self, request: AddSpot.Request, response: AddSpot.Response):
        spot = request.spot
        title = spot.title.strip()
        if not title:
            response.success = False
            response.message = "Add spot request rejected: empty title"
            self.get_logger().warn(response.message)
            return response

        new_spot = Spot()
        new_spot.title = title
        new_spot.pose_x = float(spot.pose_x)
        new_spot.pose_y = float(spot.pose_y)
        new_spot.pose_z = float(spot.pose_z)
        new_spot.pose_theta = float(spot.pose_theta)

        self.spots_dict[title] = new_spot
        if not self._persist_spots_yaml():
            self.spots_dict.pop(title, None)
            response.success = False
            response.message = f"Add spot failed: could not write spots.yaml for '{title}'"
            self.get_logger().error(response.message)
            return response

        response.success = True
        response.message = f"Spot '{title}' saved successfully."
        self.get_logger().info(response.message)
        self.publish_markers()
        return response

    def handle_delete_spot(self, request: DeleteSpot.Request, response: DeleteSpot.Response):
        title = request.title.strip()
        if not title:
            response.success = False
            response.message = "Delete spot request rejected: empty title"
            self.get_logger().warn(response.message)
            return response

        spot_to_del = self._find_spot_case_insensitive(title)
        if spot_to_del is None:
            response.success = False
            response.message = f"Delete spot failed: '{title}' not found"
            self.get_logger().warn(response.message)
            return response

        actual_title = spot_to_del.title
        removed_spot = self.spots_dict.pop(actual_title, None)

        if not self._persist_spots_yaml():
            self.spots_dict[actual_title] = removed_spot
            response.success = False
            response.message = f"Delete spot failed: could not update spots.yaml for '{title}'"
            self.get_logger().error(response.message)
            return response

        response.success = True
        response.message = f"Spot '{actual_title}' deleted successfully."
        self.get_logger().info(response.message)
        self.publish_markers(clear_first=True)
        return response

    def handle_navigate_to_spot(self, request: NavigateToSpot.Request, response: NavigateToSpot.Response):
        title = request.title.strip()
        if not title:
            response.success = False
            response.message = "Navigate spot request rejected: empty title"
            return response

        spot = self._find_spot_case_insensitive(title)
        if spot is None:
            response.success = False
            response.message = f"Spot '{title}' not found"
            self.get_logger().warn(response.message)
            return response

        if not self.nav_client.wait_for_server(timeout_sec=3.0):
            response.success = False
            response.message = "Nav2 action server 'navigate_to_pose' is not available"
            self.get_logger().error(response.message)
            return response

        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = float(spot.pose_x)
        goal.pose.position.y = float(spot.pose_y)
        goal.pose.position.z = float(spot.pose_z)

        half_yaw = float(spot.pose_theta) * 0.5
        goal.pose.orientation.z = math.sin(half_yaw)
        goal.pose.orientation.w = math.cos(half_yaw)

        self.get_logger().info(
            f"🚀 Dispatching Nav2 goal for spot '{spot.title}': x={spot.pose_x:.2f}, y={spot.pose_y:.2f}, theta={spot.pose_theta:.2f}"
        )
        
        # Dispatch goal asynchronously without blocking the service thread
        send_goal_future = self.nav_client.send_goal_async(goal)

        def goal_accepted_cb(future):
            goal_handle = future.result()
            if not goal_handle or not goal_handle.accepted:
                self.get_logger().error(f"❌ Nav2 rejected goal for spot '{spot.title}'")
            else:
                self.get_logger().info(f"✅ Nav2 goal accepted for spot '{spot.title}'")

        send_goal_future.add_done_callback(goal_accepted_cb)

        response.success = True
        response.message = f"Navigating to spot '{spot.title}'"
        return response


def main(args=None):
    rclpy.init(args=args)
    node = SpotsManagerNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
