#!/usr/bin/env python3
import time

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sanad_interfaces.msg import Device, Room
from sanad_interfaces.srv import (
    DeleteDevice,
    GetAllRooms,
    GetRoom,
    RegisterDevice,
    UpdateRoom,
)


class SmartHomeDeviceNode(Node):
    def __init__(self):
        super().__init__("smart_home_device_node")
        self.callback_group = ReentrantCallbackGroup()
        self.get_room_client = self.create_client(
            GetRoom,
            "rooms/get",
            callback_group=self.callback_group,
        )
        self.get_all_rooms_client = self.create_client(
            GetAllRooms,
            "rooms/get_all",
            callback_group=self.callback_group,
        )
        self.update_room_client = self.create_client(
            UpdateRoom,
            "rooms/update",
            callback_group=self.callback_group,
        )
        self.register_device_srv = self.create_service(
            RegisterDevice,
            "devices/register",
            self.handle_register_device,
            callback_group=self.callback_group,
        )
        self.delete_device_srv = self.create_service(
            DeleteDevice,
            "devices/delete",
            self.handle_delete_device,
            callback_group=self.callback_group,
        )
        self.get_logger().info("Smart Home Device Node has been started.")

    def _device_exists_in_room(self, device_id: str, room: Room):
        for device in room.devices:
            if str(device.id).strip() == device_id:
                return True
        return False

    def _wait_for_future(self, future):
        while rclpy.ok() and not future.done():
            time.sleep(0.05)
        return future.result()

    def _get_room(self, room_name: str):
        request = GetRoom.Request()
        request.name = room_name

        if not self.get_room_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("rooms/get service is not available")
            return None

        future = self.get_room_client.call_async(request)
        response = self._wait_for_future(future)
        if response is None or not response.success:
            return None
        return response.room

    def _get_all_rooms(self):
        request = GetAllRooms.Request()

        if not self.get_all_rooms_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("rooms/get_all service is not available")
            return []

        future = self.get_all_rooms_client.call_async(request)
        response = self._wait_for_future(future)
        if response is None:
            return []
        return list(response.rooms)

    def _update_room(self, room: Room):
        request = UpdateRoom.Request()
        request.room = room

        if not self.update_room_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("rooms/update service is not available")
            return False

        future = self.update_room_client.call_async(request)
        response = self._wait_for_future(future)
        return bool(response and response.success)

    def _find_room_containing_device(self, device_id: str):
        for room in self._get_all_rooms():
            for device in room.devices:
                if str(device.id).strip() == device_id:
                    return room
        return None

    def handle_register_device(
        self, request: RegisterDevice.Request, response: RegisterDevice.Response
    ):
        room_name = request.room_name.strip()
        device_id = request.device.id.strip()

        if not room_name:
            response.success = False
            self.get_logger().warn("Register device request rejected: empty room_name")
            return response

        if not device_id:
            response.success = False
            self.get_logger().warn("Register device request rejected: empty device.id")
            return response

        room = self._get_room(room_name)
        if room is None:
            response.success = False
            self.get_logger().warn(
                f"Register device failed: room '{room_name}' was not found"
            )
            return response

        if self._device_exists_in_room(device_id, room):
            response.success = False
            self.get_logger().warn(
                f"Register device failed: device id '{device_id}' already exists in room '{room_name}'"
            )
            return response

        room.devices.append(request.device)
        if not self._update_room(room):
            response.success = False
            self.get_logger().error(
                f"Register device failed: could not update room '{room_name}'"
            )
            return response

        response.success = True
        self.get_logger().info(f"Registered device '{device_id}' in room '{room_name}'")
        return response

    def handle_delete_device(
        self, request: DeleteDevice.Request, response: DeleteDevice.Response
    ):
        device_id = request.id.strip()

        if not device_id:
            response.success = False
            self.get_logger().warn("Delete device request rejected: empty id")
            return response

        room = self._find_room_containing_device(device_id)
        if room is None:
            response.success = False
            self.get_logger().warn(
                f"Delete device failed: device id '{device_id}' was not found"
            )
            return response

        updated_devices = [
            device for device in room.devices if str(device.id).strip() != device_id
        ]
        if len(updated_devices) == len(room.devices):
            response.success = False
            self.get_logger().warn(
                f"Delete device failed: device id '{device_id}' was not found in room '{room.room_name}'"
            )
            return response

        room.devices = updated_devices
        if not self._update_room(room):
            response.success = False
            self.get_logger().error(
                f"Delete device failed: could not update room '{room.room_name}'"
            )
            return response

        response.success = True
        self.get_logger().info(
            f"Deleted device '{device_id}' from room '{room.room_name}'"
        )
        return response


def main(args=None):
    rclpy.init(args=args)
    node = SmartHomeDeviceNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
