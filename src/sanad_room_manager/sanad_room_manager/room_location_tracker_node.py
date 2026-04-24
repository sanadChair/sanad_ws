#!/usr/bin/env python3
import os

import yaml
from ament_index_python import get_package_share_directory
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


sanad_room_manager_pkg = get_package_share_directory("sanad_room_manager")


class RoomLocationTrackerNode(Node):
    def __init__(self):
        super().__init__("room_location_tracker_node")

        self.declare_parameter("rooms_yaml_path", "")
        self.declare_parameter("robot_odom_topic", "/odom")
        self.declare_parameter("location_topic", "/rooms/location")
        self.declare_parameter("heartbeat_seconds", 30.0)

        self.rooms_yaml_path = self._resolve_rooms_yaml_path()
        self.rooms_bounds = self._load_rooms_bounds()

        odom_topic = str(self.get_parameter("robot_odom_topic").value)
        location_topic = str(self.get_parameter("location_topic").value)
        self.heartbeat_seconds = float(self.get_parameter("heartbeat_seconds").value)

        self.location_pub = self.create_publisher(String, location_topic, 10)
        self.odom_sub = self.create_subscription(
            Odometry,
            odom_topic,
            self._odom_callback,
            10,
        )

        # Use a periodic timer to publish a heartbeat if the room did not change.
        self.heartbeat_timer = self.create_timer(1.0, self._heartbeat_tick)

        self.current_room = ""
        self.last_publish_time_sec = self._now_sec()

        self.get_logger().info(
            f"Room Location Tracker started. Watching '{odom_topic}', publishing to '{location_topic}', heartbeat={self.heartbeat_seconds:.1f}s"
        )

    def _resolve_rooms_yaml_path(self) -> str:
        configured_path = str(self.get_parameter("rooms_yaml_path").value).strip()
        if configured_path:
            return configured_path

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

    def _load_rooms_bounds(self):
        rooms_data = self._load_rooms_yaml()
        bounds = {}
        for room_name, room_entry in rooms_data.items():
            coordinates = room_entry.get("coordinates", {})
            try:
                bounds[str(room_name)] = {
                    "min_x": float(coordinates.get("min_x", 0.0)),
                    "max_x": float(coordinates.get("max_x", 0.0)),
                    "min_y": float(coordinates.get("min_y", 0.0)),
                    "max_y": float(coordinates.get("max_y", 0.0)),
                }
            except (TypeError, ValueError):
                self.get_logger().warn(
                    f"Skipping room '{room_name}' because coordinates are invalid"
                )

        self.get_logger().info(f"Loaded bounds for {len(bounds)} rooms")
        return bounds

    def _now_sec(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def _estimate_room(self, x: float, y: float) -> str:
        for room_name, room_bounds in self.rooms_bounds.items():
            if (
                room_bounds["min_x"] <= x <= room_bounds["max_x"]
                and room_bounds["min_y"] <= y <= room_bounds["max_y"]
            ):
                return room_name
        return "unknown"

    def _publish_room(self, room_name: str, reason: str) -> None:
        msg = String()
        msg.data = room_name
        self.location_pub.publish(msg)
        self.last_publish_time_sec = self._now_sec()
        self.get_logger().info(f"Published room='{room_name}' ({reason})")

    def _odom_callback(self, msg: Odometry) -> None:
        x = float(msg.pose.pose.position.x)
        y = float(msg.pose.pose.position.y)
        estimated_room = self._estimate_room(x, y)

        if estimated_room != self.current_room:
            self.current_room = estimated_room
            self._publish_room(estimated_room, "room_changed")

    def _heartbeat_tick(self) -> None:
        if not self.current_room:
            return

        elapsed = self._now_sec() - self.last_publish_time_sec
        if elapsed >= self.heartbeat_seconds:
            self._publish_room(self.current_room, "heartbeat")


def main(args=None):
    rclpy.init(args=args)
    node = RoomLocationTrackerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()