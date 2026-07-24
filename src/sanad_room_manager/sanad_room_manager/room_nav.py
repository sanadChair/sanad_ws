#!/usr/bin/env python3
import math
import os
import time

import yaml
from ament_index_python import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sanad_interfaces.msg import Room
from sanad_interfaces.srv import NavigateToRoom


sanad_room_manager_pkg = get_package_share_directory("sanad_room_manager")


class RoomNavNode(Node):
    def __init__(self):
        super().__init__("room_nav_node")
        self.callback_group = ReentrantCallbackGroup()
        self.rooms_yaml_path = self._resolve_rooms_yaml_path()
        self.rooms_dict = self._load_rooms_dict()
        self.nav_client = ActionClient(
            self,
            NavigateToPose,
            "navigate_to_pose",
            callback_group=self.callback_group,
        )
        self.navigate_srv = self.create_service(
            NavigateToRoom,
            "rooms/navigate",
            self.handle_navigate_to_room,
            callback_group=self.callback_group,
        )
        self.get_logger().info("Room Nav Node has been started.")

    def _resolve_rooms_yaml_path(self):
        env_yaml_path = os.environ.get("SANAD_ROOMS_YAML_PATH")
        if env_yaml_path:
            return os.path.abspath(env_yaml_path)

        workspace_yaml_path = os.path.abspath(
            os.path.join(
                os.getcwd(),
                "src",
                "sanad_room_manager",
                "config",
                "rooms.yaml",
            )
        )
        if os.path.isfile(workspace_yaml_path):
            return workspace_yaml_path

        source_yaml_path = os.path.abspath(
            os.path.join(os.path.dirname(__file__), "..", "config", "rooms.yaml")
        )
        if os.path.isfile(source_yaml_path):
            return source_yaml_path
        return os.path.join(sanad_room_manager_pkg, "config", "rooms.yaml")

    def _load_rooms_yaml(self):
        try:
            with open(self.rooms_yaml_path, "r", encoding="utf-8") as yaml_file:
                data = yaml.safe_load(yaml_file) or {}
        except Exception as exc:
            self.get_logger().error(
                f"Failed to load rooms YAML '{self.rooms_yaml_path}': {exc}"
            )
            return {}

        if isinstance(data.get("House"), dict):
            return data["House"]

        if isinstance(data, dict):
            return data

        self.get_logger().error("Invalid rooms YAML format: expected a map")
        return {}

    def _build_room_msg(self, room_name: str, room_entry: dict) -> Room:
        coordinates = room_entry.get("coordinates", {})
        pose = coordinates.get("pose", {})

        room = Room()
        room.id = str(room_entry.get("id", ""))
        room.name = str(room_entry.get("name") or room_name)
        room.min_x = float(coordinates.get("min_x", 0.0))
        room.max_x = float(coordinates.get("max_x", 0.0))
        room.min_y = float(coordinates.get("min_y", 0.0))
        room.max_y = float(coordinates.get("max_y", 0.0))
        room.theta = float(coordinates.get("theta", 0.0))
        room.pose_x = float(pose.get("x", 0.0))
        room.pose_y = float(pose.get("y", 0.0))
        room.pose_theta = float(pose.get("theta", 0.0))
        return room

    def _load_rooms_dict(self):
        rooms_data = self._load_rooms_yaml()
        rooms_dict = {}

        for room_name, room_entry in rooms_data.items():
            rooms_dict[str(room_name)] = self._build_room_msg(
                str(room_name), room_entry
            )

        self.get_logger().info(f"Loaded {len(rooms_dict)} rooms into local cache")
        return rooms_dict

    def _room_to_pose_stamped(self, room: Room) -> PoseStamped:
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(room.pose_x)
        pose.pose.position.y = float(room.pose_y)
        pose.pose.position.z = 0.0

        half_yaw = float(room.pose_theta) * 0.5
        pose.pose.orientation.z = math.sin(half_yaw)
        pose.pose.orientation.w = math.cos(half_yaw)
        return pose

    def _wait_for_result(self, future):
        while rclpy.ok() and not future.done():
            time.sleep(0.1)
        return future.result()

    def handle_navigate_to_room(
        self, request: NavigateToRoom.Request, response: NavigateToRoom.Response
    ):
        room_id = request.id.strip()
        if not room_id:
            response.success = False
            self.get_logger().warn("Navigate request rejected: empty room id")
            return response

        self.rooms_dict = self._load_rooms_dict()
        room = self.rooms_dict.get(room_id)
        if room is None:
            for candidate in self.rooms_dict.values():
                if candidate.id == room_id or candidate.name == room_id:
                    room = candidate
                    break

        if room is None:
            response.success = False
            self.get_logger().warn(
                f"Navigate request failed: room '{room_id}' not found"
            )
            return response

        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            response.success = False
            self.get_logger().error(
                "Nav2 action server 'navigate_to_pose' is not available"
            )
            return response

        goal = NavigateToPose.Goal()
        goal.pose = self._room_to_pose_stamped(room)

        self.get_logger().info(
            f"Sending room '{room.name or room.id}' to Nav2 goal: x={room.pose_x:.2f}, y={room.pose_y:.2f}, theta={room.pose_theta:.2f}"
        )
        send_goal_future = self.nav_client.send_goal_async(goal)
        goal_handle = self._wait_for_result(send_goal_future)
        if goal_handle is None or not goal_handle.accepted:
            response.success = False
            self.get_logger().error(f"Nav2 rejected goal for room '{room.name or room.id}'")
            return response

        result_future = goal_handle.get_result_async()
        result = self._wait_for_result(result_future)
        if result is None:
            response.success = False
            self.get_logger().error(f"No result returned for room '{room.name or room.id}'")
            return response

        if result.result and result.status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(
                f"Navigation to room '{room.name or room.id}' completed successfully"
            )
            response.success = True
            return response

        self.get_logger().error(
            f"Navigation to room '{room.name or room.id}' failed with status {result.status}"
        )
        response.success = False
        return response


def main(args=None):
    rclpy.init(args=args)
    node = RoomNavNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
