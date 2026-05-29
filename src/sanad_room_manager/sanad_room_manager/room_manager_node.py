#!/usr/bin/env python3
import json
import os
from typing import Any

import rclpy
import yaml
from ament_index_python import get_package_share_directory
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sanad_interfaces.msg import Device, DeviceEvent, Room, RoomEvent
from sanad_interfaces.srv import GetAllDevices, GetAllRooms


sanad_room_manager_pkg = get_package_share_directory("sanad_room_manager")


class RoomManagerNode(Node):
    def __init__(self):
        super().__init__("room_manager_node")
        self.callback_group = ReentrantCallbackGroup()
        self.rooms_yaml_path = self._resolve_rooms_yaml_path()
        self.rooms_by_id = {}
        self.room_order = []
        self.known_devices = {}
        self.sync_in_progress = False
        self.initial_sync_complete = False

        self.get_all_rooms_client = self.create_client(
            GetAllRooms,
            "/room/getAllRooms",
            callback_group=self.callback_group,
        )
        self.get_all_devices_client = self.create_client(
            GetAllDevices,
            "/smarthome/getAllDevices",
            callback_group=self.callback_group,
        )
        self.create_subscription(
            RoomEvent,
            "/Room/RoomEvents",
            self.handle_room_event,
            10,
            callback_group=self.callback_group,
        )
        self.create_subscription(
            DeviceEvent,
            "/SmartHome/DeviceEvents",
            self.handle_device_event,
            10,
            callback_group=self.callback_group,
        )

        self.startup_retry_timer = self.create_timer(
            5.0,
            self.sync_from_server,
            callback_group=self.callback_group,
        )
        self.resync_timer = self.create_timer(
            60.0,
            self.sync_from_server,
            callback_group=self.callback_group,
        )
        self.get_logger().info(f"Rooms YAML path: {self.rooms_yaml_path}")
        self.get_logger().info(
            "Listening for room events on /Room/RoomEvents and device events "
            "on /SmartHome/DeviceEvents"
        )
        self.sync_from_server()
        self.get_logger().info("Room Manager sync node has been started.")

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

    def sync_from_server(self):
        if self.sync_in_progress:
            return

        if not self.get_all_rooms_client.service_is_ready():
            if not self.get_all_rooms_client.wait_for_service(timeout_sec=0.1):
                self.get_logger().warn(
                    "Waiting for NodeJS service /room/getAllRooms..."
                )
                return

        if not self.get_all_devices_client.service_is_ready():
            if not self.get_all_devices_client.wait_for_service(timeout_sec=0.1):
                self.get_logger().warn(
                    "Waiting for NodeJS service /smarthome/getAllDevices..."
                )
                return

        self.get_logger().info(
            "Requesting rooms from /room/getAllRooms and devices from "
            "/smarthome/getAllDevices"
        )
        self.sync_in_progress = True
        rooms_future = self.get_all_rooms_client.call_async(GetAllRooms.Request())
        devices_future = self.get_all_devices_client.call_async(GetAllDevices.Request())
        pending = {"rooms": None, "devices": None}

        def finish_if_ready():
            if pending["rooms"] is None or pending["devices"] is None:
                return

            try:
                self._replace_snapshot(pending["rooms"], pending["devices"])
                self.initial_sync_complete = True
                self.get_logger().info(
                    f"Synced {len(self.rooms_by_id)} rooms and "
                    f"{len(self.known_devices)} devices from NodeJS"
                )
            finally:
                self.sync_in_progress = False

        def rooms_done(future):
            try:
                response = future.result()
                pending["rooms"] = list(response.rooms) if response else []
            except Exception as exc:
                self.sync_in_progress = False
                self.get_logger().error(f"Failed to sync rooms from NodeJS: {exc}")
                return
            finish_if_ready()

        def devices_done(future):
            try:
                response = future.result()
                pending["devices"] = list(response.devices) if response else []
            except Exception as exc:
                self.sync_in_progress = False
                self.get_logger().error(f"Failed to sync devices from NodeJS: {exc}")
                return
            finish_if_ready()

        rooms_future.add_done_callback(rooms_done)
        devices_future.add_done_callback(devices_done)

    def _replace_snapshot(self, rooms: list[Room], devices: list[Device]):
        self.rooms_by_id = {}
        self.room_order = []
        self.known_devices = {}

        for device in devices:
            device_dict = self._device_msg_to_dict(device)
            if device_dict["id"]:
                self.known_devices[device_dict["id"]] = device_dict
                self.get_logger().info(
                    f"Device snapshot: {self._device_label(device_dict)}"
                )

        for room in rooms:
            room_dict = self._room_msg_to_dict(room)
            room_key = self._room_key(room_dict)
            self.rooms_by_id[room_key] = room_dict
            self.room_order.append(room_key)
            self.get_logger().info(
                f"Room snapshot: {self._room_label(room_dict)} "
                f"with {len(room_dict['devices'])} devices"
            )

            for device in room_dict["devices"]:
                device_id = device.get("id", "")
                if device_id:
                    self.known_devices[device_id] = device
                    self.get_logger().info(
                        f"  Room device: {self._device_label(device)} "
                        f"in {self._room_label(room_dict)}"
                    )

        self._persist_rooms_yaml()

    def _room_msg_to_dict(self, room: Room):
        return {
            "id": str(room.id),
            "name": str(room.name),
            "min_x": float(room.min_x),
            "max_x": float(room.max_x),
            "min_y": float(room.min_y),
            "max_y": float(room.max_y),
            "theta": float(room.theta),
            "pose_x": float(room.pose_x),
            "pose_y": float(room.pose_y),
            "pose_theta": float(room.pose_theta),
            "devices": [self._device_msg_to_dict(device) for device in room.devices],
        }

    def _device_msg_to_dict(self, device: Device):
        return {
            "id": str(device.id),
            "name": str(device.name),
            "control_type": str(device.control_type),
            "state": int(device.state),
            "connected": bool(device.connected),
            "created_at": str(device.created_at),
            "updated_at": str(device.updated_at),
        }

    def _persist_rooms_yaml(self):
        serialized_rooms = {}
        for room_key in self.room_order:
            room = self.rooms_by_id.get(room_key)
            if room is None:
                continue
            serialized_rooms[self._yaml_room_key(room)] = self._room_to_yaml_entry(room)

        try:
            with open(self.rooms_yaml_path, "w", encoding="utf-8") as yaml_file:
                yaml.safe_dump(
                    serialized_rooms,
                    yaml_file,
                    sort_keys=False,
                    default_flow_style=False,
                )
            device_count = sum(
                len(room.get("devices", [])) for room in self.rooms_by_id.values()
            )
            self.get_logger().info(
                f"Wrote {len(serialized_rooms)} rooms and {device_count} room devices "
                f"to {self.rooms_yaml_path}"
            )
        except Exception as exc:
            self.get_logger().error(
                f"Failed to persist rooms YAML '{self.rooms_yaml_path}': {exc}"
            )

    def _room_to_yaml_entry(self, room: dict[str, Any]):
        return {
            "id": room.get("id", ""),
            "name": room.get("name", ""),
            "coordinates": {
                "min_x": float(room.get("min_x", 0.0)),
                "max_x": float(room.get("max_x", 0.0)),
                "min_y": float(room.get("min_y", 0.0)),
                "max_y": float(room.get("max_y", 0.0)),
                "theta": float(room.get("theta", 0.0)),
                "pose": {
                    "x": float(room.get("pose_x", 0.0)),
                    "y": float(room.get("pose_y", 0.0)),
                    "theta": float(room.get("pose_theta", 0.0)),
                },
            },
            "devices": [
                self._device_to_yaml_entry(device)
                for device in room.get("devices", [])
                if device.get("id")
            ],
        }

    def _device_to_yaml_entry(self, device: dict[str, Any]):
        return {
            "id": str(device.get("id", "")),
            "name": str(device.get("name", "")),
            "control_type": str(device.get("control_type", "")),
            "state": int(device.get("state") or 0),
            "connected": bool(device.get("connected", False)),
            "created_at": str(device.get("created_at", "")),
            "updated_at": str(device.get("updated_at", "")),
        }

    def handle_room_event(self, msg: RoomEvent):
        room_payload = self._parse_event_payload(msg.room)
        event = msg.event.strip()
        if not isinstance(room_payload, dict):
            self.get_logger().warn(f"Ignoring malformed room event: {event}")
            return

        self.get_logger().info(
            f"Room event {event}: {self._payload_room_label(room_payload)}"
        )
        if event in ("NEW_ROOM", "UPDATE"):
            self._upsert_room_from_payload(room_payload)
            self._persist_rooms_yaml()
            return

        if event == "DELETED":
            if self._delete_room_from_payload(room_payload):
                self._persist_rooms_yaml()
            return

        self.get_logger().warn(f"Ignoring unsupported room event: {event}")

    def handle_device_event(self, msg: DeviceEvent):
        device_payload = self._parse_event_payload(msg.device)
        event = msg.event.strip()
        self.get_logger().info(
            f"Device event {event}: {self._payload_device_label(device_payload)}"
        )

        if event == "DISCONNECTED":
            changed = self._mark_or_remove_disconnected_device(device_payload)
            if changed:
                self._persist_rooms_yaml()
            else:
                self.get_logger().info(
                    "Disconnected device was not in the room cache; requesting full resync"
                )
                self.sync_from_server()
            return

        if event in ("NEW_DEVICE", "ADD", "UPDATE"):
            changed = self._upsert_device_from_payload(device_payload)
            if changed:
                self._persist_rooms_yaml()
            else:
                self.get_logger().info(
                    "Device event did not include enough room context to update YAML; "
                    "requesting full resync"
                )
                self.sync_from_server()
            return

        self.get_logger().warn(f"Ignoring unsupported device event: {event}")

    def _parse_event_payload(self, payload: str):
        try:
            return json.loads(payload)
        except Exception as exc:
            self.get_logger().warn(f"Failed to parse event payload as JSON: {exc}")
            return None

    def _upsert_room_from_payload(self, payload: dict[str, Any]):
        room = self._normalize_room_payload(payload)
        room_key = self._room_key(room)

        previous_key = self._find_room_key(room)
        if previous_key and previous_key != room_key:
            self.rooms_by_id.pop(previous_key, None)
            self.room_order = [
                room_key if key == previous_key else key for key in self.room_order
            ]
        elif room_key not in self.room_order:
            self.room_order.append(room_key)

        self.rooms_by_id[room_key] = room
        self.get_logger().info(
            f"Cached room {self._room_label(room)} with {len(room['devices'])} devices"
        )
        for device in room["devices"]:
            if device.get("id"):
                self.known_devices[device["id"]] = device
                self.get_logger().info(
                    f"  Cached device {self._device_label(device)} "
                    f"in {self._room_label(room)}"
                )

    def _delete_room_from_payload(self, payload: dict[str, Any]):
        room_key = self._find_room_key(payload)
        if room_key is None:
            self.get_logger().warn(
                f"Room delete event did not match local cache: {payload}"
            )
            return False

        self.rooms_by_id.pop(room_key, None)
        self.room_order = [key for key in self.room_order if key != room_key]
        self.get_logger().info(f"Removed room from cache: {room_key}")
        return True

    def _upsert_device_from_payload(self, payload: Any):
        if not isinstance(payload, dict):
            self.get_logger().warn(f"Ignoring malformed device event payload: {payload}")
            return False

        device = self._normalize_device_payload(payload)
        device_id = device.get("id", "")
        if not device_id:
            self.get_logger().warn("Ignoring device event without id")
            return False

        self.known_devices[device_id] = device
        target_room_key = self._device_room_key(payload)
        changed = False

        if target_room_key:
            changed = self._remove_device_from_rooms(device_id, except_key=target_room_key)
            room = self.rooms_by_id.get(target_room_key)
            if room is None:
                self.get_logger().warn(
                    f"Device event references unknown room: {target_room_key}"
                )
                return changed
            updated = self._upsert_device_in_room(room, device) or changed
            if updated:
                self.get_logger().info(
                    f"Cached device {self._device_label(device)} "
                    f"in {self._room_label(room)}"
                )
            return updated

        for room in self.rooms_by_id.values():
            if self._upsert_device_in_room(room, device, only_existing=True):
                self.get_logger().info(
                    f"Updated existing device {self._device_label(device)} "
                    f"in {self._room_label(room)}"
                )
                return True

        self.get_logger().debug(
            f"Device '{device_id}' has no room assignment in event; cached only"
        )
        return changed

    def _mark_or_remove_disconnected_device(self, payload: Any):
        if isinstance(payload, str):
            return self._remove_device_from_rooms(payload)

        if not isinstance(payload, dict):
            self.get_logger().warn(
                f"Ignoring malformed disconnected device payload: {payload}"
            )
            return False

        device = self._normalize_device_payload(payload)
        device["connected"] = False
        device_id = device.get("id", "")
        if not device_id:
            return False

        self.known_devices[device_id] = device
        for room in self.rooms_by_id.values():
            if self._upsert_device_in_room(room, device, only_existing=True):
                self.get_logger().info(
                    f"Marked device disconnected: {self._device_label(device)} "
                    f"in {self._room_label(room)}"
                )
                return True
        return False

    def _upsert_device_in_room(
        self,
        room: dict[str, Any],
        device: dict[str, Any],
        only_existing: bool = False,
    ):
        devices = room.setdefault("devices", [])
        device_id = device.get("id", "")
        for index, existing_device in enumerate(devices):
            if existing_device.get("id") == device_id:
                devices[index] = {**existing_device, **device}
                return True

        if only_existing:
            return False

        devices.append(device)
        return True

    def _remove_device_from_rooms(self, device_id: str, except_key: str | None = None):
        changed = False
        for room_key, room in self.rooms_by_id.items():
            if room_key == except_key:
                continue
            devices = room.get("devices", [])
            filtered_devices = [
                device for device in devices if device.get("id") != device_id
            ]
            if len(filtered_devices) != len(devices):
                room["devices"] = filtered_devices
                self.get_logger().info(
                    f"Removed device '{device_id}' from {self._room_label(room)}"
                )
                changed = True
        return changed

    def _normalize_room_payload(self, payload: dict[str, Any]):
        raw_devices = payload.get("devices") or []
        return {
            "id": str(payload.get("id") or ""),
            "name": str(payload.get("name") or payload.get("room_name") or ""),
            "min_x": self._float_value(payload.get("min_x")),
            "max_x": self._float_value(payload.get("max_x")),
            "min_y": self._float_value(payload.get("min_y")),
            "max_y": self._float_value(payload.get("max_y")),
            "theta": self._float_value(payload.get("theta")),
            "pose_x": self._float_value(payload.get("pose_x")),
            "pose_y": self._float_value(payload.get("pose_y")),
            "pose_theta": self._float_value(payload.get("pose_theta")),
            "devices": [
                self._normalize_device_payload(device)
                for device in raw_devices
                if isinstance(device, dict)
            ],
        }

    def _normalize_device_payload(self, payload: dict[str, Any]):
        return {
            "id": str(payload.get("id") or payload.get("deviceId") or ""),
            "name": str(payload.get("name") or ""),
            "control_type": str(payload.get("control_type") or payload.get("type") or ""),
            "state": self._int_value(payload.get("state")),
            "connected": bool(payload.get("connected", False)),
            "created_at": str(payload.get("created_at") or ""),
            "updated_at": str(payload.get("updated_at") or ""),
        }

    def _device_room_key(self, payload: dict[str, Any]):
        room_payload = payload.get("room")
        room_id = payload.get("room_id") or payload.get("roomId")
        room_name = payload.get("room_name") or payload.get("roomName")

        if isinstance(room_payload, dict):
            room_id = room_id or room_payload.get("id")
            room_name = room_name or room_payload.get("name")
        elif isinstance(room_payload, str):
            room_id = room_id or room_payload

        return self._find_room_key({"id": room_id, "name": room_name})

    def _find_room_key(self, payload: dict[str, Any]):
        requested_id = str(payload.get("id") or "").strip()
        requested_name = str(payload.get("name") or payload.get("room_name") or "").strip()

        for room_key, room in self.rooms_by_id.items():
            if requested_id and (room_key == requested_id or room.get("id") == requested_id):
                return room_key
            if requested_name and (
                room_key == requested_name or room.get("name") == requested_name
            ):
                return room_key
        return None

    def _room_key(self, room: dict[str, Any]):
        return str(room.get("id") or room.get("name") or "unknown").strip()

    def _yaml_room_key(self, room: dict[str, Any]):
        return str(room.get("name") or room.get("id") or "unknown").strip()

    def _room_label(self, room: dict[str, Any]):
        name = str(room.get("name") or "").strip()
        room_id = str(room.get("id") or "").strip()
        if name and room_id:
            return f"{name} ({room_id})"
        return name or room_id or "unknown room"

    def _device_label(self, device: dict[str, Any]):
        name = str(device.get("name") or "").strip()
        device_id = str(device.get("id") or "").strip()
        if name and device_id:
            return f"{name} ({device_id})"
        return name or device_id or "unknown device"

    def _payload_room_label(self, payload: Any):
        if not isinstance(payload, dict):
            return str(payload)
        return self._room_label(
            {
                "id": payload.get("id"),
                "name": payload.get("name") or payload.get("room_name"),
            }
        )

    def _payload_device_label(self, payload: Any):
        if not isinstance(payload, dict):
            return str(payload)
        return self._device_label(
            {
                "id": payload.get("id") or payload.get("deviceId"),
                "name": payload.get("name"),
            }
        )

    def _float_value(self, value: Any):
        try:
            return float(value)
        except (TypeError, ValueError):
            return 0.0

    def _int_value(self, value: Any):
        try:
            return int(value)
        except (TypeError, ValueError):
            return 0


def main(args=None):
    rclpy.init(args=args)
    node = RoomManagerNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
