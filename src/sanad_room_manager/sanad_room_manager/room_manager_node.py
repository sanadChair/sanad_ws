#!/usr/bin/env python3
import os
import yaml
from ament_index_python import get_package_share_directory
from sanad_interfaces.msg import Room
from sanad_interfaces.srv import AddRoom, DeleteRoom, GetAllRooms, GetRoom, UpdateRoom
import rclpy
from rclpy.node import Node


sanad_room_manager_pkg = get_package_share_directory("sanad_room_manager")


class RoomManagerNode(Node):
    def __init__(self):
        super().__init__("room_manager_node")
        self.declare_parameter("use_sim_time", False)
        self.rooms_yaml_path = self._resolve_rooms_yaml_path()
        self.rooms_dict = self._load_rooms_dict()
        self.get_srv = self.create_service(GetRoom, "rooms/get", self.callback_get)

        self.add_srv = self.create_service(AddRoom, "rooms/add", self.handle_add_room)
        self.update_srv = self.create_service(
            UpdateRoom, "rooms/update", self.handle_update_room
        )
        self.delete_srv = self.create_service(
            DeleteRoom, "rooms/delete", self.handle_delete_room
        )
        self.get_all_srv = self.create_service(
            GetAllRooms, "rooms/get_all", self.handle_get_all_rooms
        )
        self.get_logger().info("Room Manager Node has been started.")

    def _resolve_rooms_yaml_path(self):
        # Prefer the source config during development so updates are visible in workspace.
        source_yaml_path = os.path.abspath(
            os.path.join(os.path.dirname(__file__), "..", "config", "rooms.yaml")
        )
        if os.path.isfile(source_yaml_path):
            return source_yaml_path

        return os.path.join(sanad_room_manager_pkg, "config", "rooms.yaml")

    def _load_rooms_dict(self):
        rooms_data = self._load_rooms_yaml()
        rooms_dict = {}

        for room_name, room_entry in rooms_data.items():
            rooms_dict[str(room_name)] = self._build_room_msg(
                str(room_name), room_entry
            )

        self.get_logger().info(f"Loaded {len(rooms_dict)} rooms into local cache")
        return rooms_dict

    def _load_rooms_yaml(self):
        try:
            with open(self.rooms_yaml_path, "r", encoding="utf-8") as yaml_file:
                data = yaml.safe_load(yaml_file) or {}
        except Exception as exc:
            self.get_logger().error(
                f"Failed to load rooms YAML '{self.rooms_yaml_path}': {exc}"
            )
            return {}

        # Support both formats:
        # 1) {House: {kitchen: {...}}}
        # 2) {kitchen: {...}}
        if isinstance(data.get("House"), dict):
            return data["House"]

        if isinstance(data, dict):
            return data

        self.get_logger().error("Invalid rooms YAML format: expected a map")
        return {}

    def _room_to_yaml_entry(self, room: Room):
        return {
            "coordinates": {
                "min_x": float(room.min_x),
                "max_x": float(room.max_x),
                "min_y": float(room.min_y),
                "max_y": float(room.max_y),
                "theta": float(room.theta),
                "pose": {
                    "x": float(room.pose_x),
                    "y": float(room.pose_y),
                    "theta": float(room.pose_theta),
                },
            }
        }

    def _persist_rooms_yaml(self):
        serialized_rooms = {}
        for room_name, room in self.rooms_dict.items():
            serialized_rooms[room_name] = self._room_to_yaml_entry(room)

        try:
            with open(self.rooms_yaml_path, "w", encoding="utf-8") as yaml_file:
                yaml.safe_dump(
                    serialized_rooms,
                    yaml_file,
                    sort_keys=False,
                    default_flow_style=False,
                )
            return True
        except Exception as exc:
            self.get_logger().error(
                f"Failed to persist rooms YAML '{self.rooms_yaml_path}': {exc}"
            )
            return False

    def _build_room_msg(self, room_name: str, room_entry: dict) -> Room:
        coordinates = room_entry.get("coordinates", {})
        pose = coordinates.get("pose", {})

        room = Room()
        room.room_name = room_name
        room.min_x = float(coordinates.get("min_x", 0.0))
        room.max_x = float(coordinates.get("max_x", 0.0))
        room.min_y = float(coordinates.get("min_y", 0.0))
        room.max_y = float(coordinates.get("max_y", 0.0))
        room.theta = float(coordinates.get("theta", 0.0))
        room.pose_x = float(pose.get("x", 0.0))
        room.pose_y = float(pose.get("y", 0.0))
        room.pose_theta = float(pose.get("theta", 0.0))
        return room

    def callback_get(self, request: GetRoom.Request, response: GetRoom.Response):
        requested_name = request.name.strip()

        cached_room = self.rooms_dict.get(requested_name)
        if cached_room is None:
            response.success = False
            self.get_logger().warn(f"Room not found: '{request.name}'")
            return response

        response.success = True
        response.room = cached_room
        return response

    def handle_get_all_rooms(
        self, request: GetAllRooms.Request, response: GetAllRooms.Response
    ):
        del request

        for room in self.rooms_dict.values():
            response.rooms.append(room)

        return response

    def handle_add_room(self, request: AddRoom.Request, response: AddRoom.Response):
        room = request.room
        room_name = room.room_name.strip()
        if not room_name:
            response.success = False
            self.get_logger().warn("Add room request rejected: empty room_name")
            return response

        if room_name in self.rooms_dict:
            response.success = False
            self.get_logger().warn(f"Add room failed: '{room_name}' already exists")
            return response

        cached_room = Room()
        cached_room.room_name = room_name
        cached_room.min_x = float(room.min_x)
        cached_room.max_x = float(room.max_x)
        cached_room.min_y = float(room.min_y)
        cached_room.max_y = float(room.max_y)
        cached_room.theta = float(room.theta)
        cached_room.pose_x = float(room.pose_x)
        cached_room.pose_y = float(room.pose_y)
        cached_room.pose_theta = float(room.pose_theta)

        self.rooms_dict[room_name] = cached_room
        if not self._persist_rooms_yaml():
            self.rooms_dict.pop(room_name, None)
            response.success = False
            self.get_logger().error(
                f"Add room failed: could not persist rooms.yaml after adding '{room_name}'"
            )
            return response

        response.success = True
        self.get_logger().info(f"Added room '{room_name}' and updated rooms.yaml")
        return response

    def handle_update_room(
        self, request: UpdateRoom.Request, response: UpdateRoom.Response
    ):
        room = request.room
        room_name = room.room_name.strip()
        if not room_name:
            response.success = False
            self.get_logger().warn("Update room request rejected: empty room_name")
            return response

        previous_room = self.rooms_dict.get(room_name)
        if previous_room is None:
            response.success = False
            self.get_logger().warn(f"Update room failed: '{room_name}' was not found")
            return response

        cached_room = Room()
        cached_room.room_name = room_name
        cached_room.min_x = float(room.min_x)
        cached_room.max_x = float(room.max_x)
        cached_room.min_y = float(room.min_y)
        cached_room.max_y = float(room.max_y)
        cached_room.theta = float(room.theta)
        cached_room.pose_x = float(room.pose_x)
        cached_room.pose_y = float(room.pose_y)
        cached_room.pose_theta = float(room.pose_theta)

        self.rooms_dict[room_name] = cached_room
        if not self._persist_rooms_yaml():
            self.rooms_dict[room_name] = previous_room
            response.success = False
            self.get_logger().error(
                f"Update room failed: could not persist rooms.yaml after updating '{room_name}'"
            )
            return response

        response.success = True
        self.get_logger().info(f"Updated room '{room_name}' and updated rooms.yaml")
        return response

    def handle_delete_room(
        self, request: DeleteRoom.Request, response: DeleteRoom.Response
    ):
        room_name = request.name.strip()
        if not room_name:
            response.success = False
            self.get_logger().warn("Delete room request rejected: empty room name")
            return response

        removed_room = self.rooms_dict.pop(room_name, None)
        if removed_room is None:
            response.success = False
            self.get_logger().warn(f"Delete room failed: '{room_name}' was not found")
            return response

        if not self._persist_rooms_yaml():
            # Roll back in-memory cache if file persistence fails.
            self.rooms_dict[room_name] = removed_room
            response.success = False
            self.get_logger().error(
                f"Delete room failed: could not persist rooms.yaml after removing '{room_name}'"
            )
            return response

        self.get_logger().info(f"Deleted room '{room_name}' and updated rooms.yaml")
        response.success = True
        return response


def main(args=None):
    rclpy.init(args=args)
    node = RoomManagerNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
