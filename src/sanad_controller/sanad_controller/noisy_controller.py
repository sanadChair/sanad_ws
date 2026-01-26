#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.constants import S_TO_NS
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
import numpy as np
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import math
def yaw_to_quaternion(yaw: float):
    half = yaw * 0.5
    return (0.0, 0.0, math.sin(half), math.cos(half))


class NoisyController(Node):

    def __init__(self):
        super().__init__("noisy_controller")
        # Defaults aligned with current workspace (gazebo + diff_drive)
        self.declare_parameter("wheel_radius", 0.1)
        self.declare_parameter("wheel_separation", 0.45)

        self.wheel_radius_ = (
            self.get_parameter("wheel_radius").get_parameter_value().double_value
        )
        self.wheel_separation_ = (
            self.get_parameter("wheel_separation").get_parameter_value().double_value
        )

        self.get_logger().info("Using wheel radius %.4f" % self.wheel_radius_)
        self.get_logger().info("Using wheel separation %.4f" % self.wheel_separation_)

        self.left_wheel_prev_pos_ = None
        self.right_wheel_prev_pos_ = None
        self.x_ = 0.0
        self.y_ = 0.0
        self.theta_ = 0.0

        # Will resolve wheel indices from JointState.name on first message
        self.left_joint_name_ = "base_left_wheel_joint"
        self.right_joint_name_ = "base_right_wheel_joint"
        self.left_index_ = None
        self.right_index_ = None

        self.joint_sub_ = self.create_subscription(
            JointState, "joint_states", self.jointCallback, 10
        )
        self.odom_pub_ = self.create_publisher(
            Odometry, "sanad_controller/odom_noisy", 10
        )

        self.speed_conversion_ = np.array(
            [
                [self.wheel_radius_ / 2, self.wheel_radius_ / 2],
                [
                    self.wheel_radius_ / self.wheel_separation_,
                    -self.wheel_radius_ / self.wheel_separation_,
                ],
            ]
        )
        self.get_logger().info("The conversion matrix is %s" % self.speed_conversion_)

        # Fill the Odometry message with invariant parameters
        self.odom_msg_ = Odometry()
        self.odom_msg_.header.frame_id = "odom"
        self.odom_msg_.child_frame_id = "base_footprint_ekf"
        self.odom_msg_.pose.pose.orientation.x = 0.0
        self.odom_msg_.pose.pose.orientation.y = 0.0
        self.odom_msg_.pose.pose.orientation.z = 0.0
        self.odom_msg_.pose.pose.orientation.w = 1.0

        # Fill the TF message
        self.br_ = TransformBroadcaster(self)
        self.transform_stamped_ = TransformStamped()
        self.transform_stamped_.header.frame_id = "odom"
        self.transform_stamped_.child_frame_id = "base_footprint_noisy"

        self.prev_time_ = None

    def jointCallback(self, msg):
        # Implements the inverse differential kinematic model
        # Given the position of the wheels, calculates their velocities
        # then calculates the velocity of the robot wrt the robot frame
        # and then converts it in the global frame and publishes the TF

        # Resolve indices on first message to avoid relying on array order
        if self.left_index_ is None or self.right_index_ is None:
            try:
                self.left_index_ = msg.name.index(self.left_joint_name_)
                self.right_index_ = msg.name.index(self.right_joint_name_)
                self.get_logger().info(
                    f"Resolved wheel indices: left={self.left_index_}, right={self.right_index_}"
                )
            except ValueError:
                self.get_logger().warn(
                    f"Wheel joint names not found in JointState. names={msg.name}"
                )
                return

        # Add noise to wheel readings
        wheel_encoder_left = msg.position[self.left_index_] + np.random.normal(0, 0.005)
        wheel_encoder_right = msg.position[self.right_index_] + np.random.normal(0, 0.005)

        # Initialize previous readings on first callback
        current_time = Time.from_msg(msg.header.stamp)
        if self.prev_time_ is None or self.left_wheel_prev_pos_ is None:
            self.left_wheel_prev_pos_ = wheel_encoder_left
            self.right_wheel_prev_pos_ = wheel_encoder_right
            self.prev_time_ = current_time
            return

        dp_left = wheel_encoder_left - self.left_wheel_prev_pos_
        dp_right = wheel_encoder_right - self.right_wheel_prev_pos_
        dt = current_time - self.prev_time_

        # Actualize the prev pose for the next iteration
        self.left_wheel_prev_pos_ = wheel_encoder_left
        self.right_wheel_prev_pos_ = wheel_encoder_right
        self.prev_time_ = current_time

        # Calculate the rotational speed of each wheel
        if dt.nanoseconds <= 0:
            return
        fi_left = dp_left / (dt.nanoseconds / S_TO_NS)
        fi_right = dp_right / (dt.nanoseconds / S_TO_NS)

        # Calculate the linear and angular velocity
        linear = (self.wheel_radius_ * fi_right + self.wheel_radius_ * fi_left) / 2
        angular = (
            self.wheel_radius_ * fi_right - self.wheel_radius_ * fi_left
        ) / self.wheel_separation_

        # Calculate the position increment
        d_s = (self.wheel_radius_ * dp_right + self.wheel_radius_ * dp_left) / 2
        d_theta = (
            self.wheel_radius_ * dp_right - self.wheel_radius_ * dp_left
        ) / self.wheel_separation_
        self.theta_ += d_theta
        self.x_ += d_s * math.cos(self.theta_)
        self.y_ += d_s * math.sin(self.theta_)

        # Compose and publish the odom message
        q = yaw_to_quaternion(self.theta_)
        # Use the time from the JointState for consistency with sim time
        self.odom_msg_.header.stamp = msg.header.stamp
        self.odom_msg_.pose.pose.position.x = self.x_
        self.odom_msg_.pose.pose.position.y = self.y_
        self.odom_msg_.pose.pose.orientation.x = q[0]
        self.odom_msg_.pose.pose.orientation.y = q[1]
        self.odom_msg_.pose.pose.orientation.z = q[2]
        self.odom_msg_.pose.pose.orientation.w = q[3]
        self.odom_msg_.twist.twist.linear.x = linear
        self.odom_msg_.twist.twist.angular.z = angular
        self.odom_pub_.publish(self.odom_msg_)

        # TF
        self.transform_stamped_.transform.translation.x = self.x_
        self.transform_stamped_.transform.translation.y = self.y_
        self.transform_stamped_.transform.rotation.x = q[0]
        self.transform_stamped_.transform.rotation.y = q[1]
        self.transform_stamped_.transform.rotation.z = q[2]
        self.transform_stamped_.transform.rotation.w = q[3]
        self.transform_stamped_.header.stamp = msg.header.stamp
        self.br_.sendTransform(self.transform_stamped_)


def main():
    rclpy.init()

    noisy_controller = NoisyController()
    rclpy.spin(noisy_controller)

    noisy_controller.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
