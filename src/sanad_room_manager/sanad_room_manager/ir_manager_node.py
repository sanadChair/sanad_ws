#!/usr/bin/env python3
import re

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

try:
    import serial
except ImportError:
    serial = None


class IrManagerNode(Node):
    def __init__(self):
        super().__init__("ir_manager_node")

        self.declare_parameter("serial_port", "/dev/ttyUSB0")
        self.declare_parameter("baud_rate", 9600)
        self.declare_parameter("read_timeout", 0.05)
        self.declare_parameter("reconnect_period", 2.0)
        self.declare_parameter("topic_name", "/ir/hexCode")

        self.serial_port = (
            self.get_parameter("serial_port").get_parameter_value().string_value
        )
        self.baud_rate = (
            self.get_parameter("baud_rate").get_parameter_value().integer_value
        )
        self.read_timeout = (
            self.get_parameter("read_timeout").get_parameter_value().double_value
        )
        self.reconnect_period = (
            self.get_parameter("reconnect_period").get_parameter_value().double_value
        )
        topic_name = self.get_parameter("topic_name").get_parameter_value().string_value

        self.publisher_ = self.create_publisher(String, topic_name, 10)
        self.serial_conn = None
        self.last_connect_attempt = 0.0

        self.timer = self.create_timer(0.02, self.read_serial)
        self.get_logger().info(
            f"IR Manager Node started. Publishing hex codes on {topic_name}"
        )

    def _connect_serial(self):
        if serial is None:
            self.get_logger().error(
                "pyserial is not installed. Install python3-serial or pyserial."
            )
            return

        now = self.get_clock().now().nanoseconds / 1e9
        if now - self.last_connect_attempt < self.reconnect_period:
            return
        self.last_connect_attempt = now

        try:
            self.serial_conn = serial.Serial(
                port=self.serial_port,
                baudrate=self.baud_rate,
                timeout=self.read_timeout,
            )
            self.get_logger().info(
                f"Connected to IR serial port {self.serial_port} at {self.baud_rate}"
            )
        except serial.SerialException as exc:
            self.serial_conn = None
            self.get_logger().warn(
                f"Could not open IR serial port {self.serial_port}: {exc}"
            )

    def _extract_hex_code(self, line: str):
        cleaned_line = line.strip()
        if not cleaned_line:
            return None

        tokens = re.split(r"[^0-9a-zA-Z]+", cleaned_line)
        for token in reversed(tokens):
            if re.fullmatch(r"(?:0x|0X)?[0-9a-fA-F]+", token):
                return token.removeprefix("0x").removeprefix("0X").upper()

        return None

    def read_serial(self):
        if self.serial_conn is None or not self.serial_conn.is_open:
            self._connect_serial()
            return

        try:
            raw_line = self.serial_conn.readline()
        except serial.SerialException as exc:
            self.get_logger().warn(f"IR serial read failed: {exc}")
            self._close_serial()
            return

        if not raw_line:
            return

        try:
            line = raw_line.decode("utf-8", errors="ignore")
        except AttributeError:
            line = str(raw_line)

        hex_code = self._extract_hex_code(line)
        if hex_code is None:
            return

        msg = String()
        msg.data = hex_code
        self.publisher_.publish(msg)

        stamp = self.get_clock().now().to_msg()
        self.get_logger().info(
            f"Published IR hexCode={hex_code} at {stamp.sec}.{stamp.nanosec:09d}"
        )

    def _close_serial(self):
        if self.serial_conn is None:
            return

        try:
            if self.serial_conn.is_open:
                self.serial_conn.close()
        except serial.SerialException as exc:
            self.get_logger().warn(f"Failed to close IR serial port: {exc}")
        finally:
            self.serial_conn = None

    def destroy_node(self):
        self._close_serial()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = IrManagerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
