#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist

import math


def quat_to_euler(x, y, z, w):
    # roll (x-axis rotation)
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    # pitch (y-axis rotation)
    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)

    # yaw (z-axis rotation)
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


class ImuToCmdVel(Node):
    def __init__(self):
        super().__init__("imu_to_cmdvel")

        self.sub = self.create_subscription(Imu, "/imu/data", self.imu_callback, 10)

        self.pub = self.create_publisher(Twist, "/cmd_vel", 10)

        self.target_ax = 0.0
        self.target_ay = 0.0
        self.calibration_duration_sec = 2.0
        self.calibration_start_time = None
        self.calibration_done = False
        self.calib_sum_ax = 0.0
        self.calib_sum_ay = 0.0
        self.calib_samples = 0

        self.kp_ax = 0.6
        self.kp_ay = 1.2
        self.accel_max = 1.0
        self.max_angular_z = 1.0
        self.max_linear_x = 1.0
        self.ax_deadband = 0.08
        self.ay_deadband = 0.08
        self.linear_slew_rate = 0.25  # m/s^2 equivalent ramp on linear.x
        self.angular_slew_rate = 0.5  # rad/s^2 equivalent ramp on angular.z

        self.last_linear_x = 0.0
        self.last_angular_z = 0.0
        self.last_cmd_time = None

        self.get_logger().info("IMU to cmd_vel node started")

    def imu_callback(self, msg):
        now = self.get_clock().now()
        ax = msg.linear_acceleration.x
        ay = msg.linear_acceleration.y

        # Use the first couple of seconds to estimate a stable initial baseline.
        if not self.calibration_done:
            if self.calibration_start_time is None:
                self.calibration_start_time = now
                self.get_logger().info(
                    f"Calibrating IMU accel baseline for {self.calibration_duration_sec:.1f}s"
                )

            self.calib_sum_ax += ax
            self.calib_sum_ay += ay
            self.calib_samples += 1

            elapsed_sec = (now - self.calibration_start_time).nanoseconds * 1e-9

            # Hold robot still during baseline capture.
            self.pub.publish(Twist())

            if elapsed_sec < self.calibration_duration_sec:
                return

            if self.calib_samples > 0:
                self.target_ax = self.calib_sum_ax / self.calib_samples
                self.target_ay = self.calib_sum_ay / self.calib_samples
            else:
                self.target_ax = ax
                self.target_ay = ay

            self.calibration_done = True
            self.get_logger().info(
                f"Calibration complete with {self.calib_samples} samples, baseline ax={self.target_ax:.3f}, baseline ay={self.target_ay:.3f}"
            )

        ax_error = ax - self.target_ax
        ay_error = ay - self.target_ay

        # Treat acceleration deviation as normalized in [-1, 1] per axis.
        ax_error = max(-self.accel_max, min(self.accel_max, ax_error))
        ay_error = max(-self.accel_max, min(self.accel_max, ay_error))

        if abs(ax_error) < self.ax_deadband:
            ax_error = 0.0
        if abs(ay_error) < self.ay_deadband:
            ay_error = 0.0

        cmd = Twist()

        # ---------------- CONTROL LOGIC ----------------
        # lateral acceleration deviation from baseline controls turn rate
        target_angular_z = max(
            -self.max_angular_z,
            min(self.max_angular_z, -self.kp_ay * ay_error),
        )

        # forward acceleration deviation from baseline controls forward/back speed
        target_linear_x = max(
            -self.max_linear_x,
            min(self.max_linear_x, self.kp_ax * ax_error),
        )

        if self.last_cmd_time is None:
            dt = 0.0
        else:
            dt = (now - self.last_cmd_time).nanoseconds * 1e-9

        max_linear_step = self.linear_slew_rate * max(0.0, dt)
        max_angular_step = self.angular_slew_rate * max(0.0, dt)

        linear_delta = target_linear_x - self.last_linear_x
        angular_delta = target_angular_z - self.last_angular_z

        linear_delta = max(-max_linear_step, min(max_linear_step, linear_delta))
        angular_delta = max(-max_angular_step, min(max_angular_step, angular_delta))

        cmd.linear.x = self.last_linear_x + linear_delta
        cmd.angular.z = self.last_angular_z + angular_delta

        self.last_linear_x = cmd.linear.x
        self.last_angular_z = cmd.angular.z
        self.last_cmd_time = now

        self.pub.publish(cmd)


def main():
    rclpy.init()
    node = ImuToCmdVel()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
