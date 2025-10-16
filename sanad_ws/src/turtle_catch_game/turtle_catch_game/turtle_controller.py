#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import math
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from my_robot_interfaces.msg import TurtleArray
from my_robot_interfaces.srv import CatchTurtle

class TurtleControllerNode(Node):
    def __init__(self):
        super().__init__("turtle_controller")
        self.target_ = None
        self.pose_ = None

        self.pose_subscriber_ = self.create_subscription(
            Pose, "turtle1/pose", self.callback_turtle_pose, 10)
        self.target_subscriber_ = self.create_subscription(
            TurtleArray, "alive_turtles", self.callback_alive_turtles, 10)
        self.cmd_vel_publisher_ = self.create_publisher(
            Twist, "turtle1/cmd_vel", 10)
        self.control_loop_timer_ = self.create_timer(0.01, self.control_loop)

    def callback_turtle_pose(self, msg):
        self.pose_ = msg

    def callback_alive_turtles(self, msg):
        if len(msg.turtles) > 0:
            self.target_ = msg.turtles[0]

    def control_loop(self):
        if self.pose_ is None or self.target_ is None:
            return

        dist_x = self.target_.x - self.pose_.x
        dist_y = self.target_.y - self.pose_.y
        distance = math.sqrt(dist_x ** 2 + dist_y ** 2)

        msg = Twist()

        if distance > 0.5:
            # Position
            msg.linear.x = 2 * distance
            # Orientation
            goal_theta = math.atan2(dist_y, dist_x)
            diff_theta = goal_theta - self.pose_.theta
            # Normalize angle
            if diff_theta > math.pi:
                diff_theta -= 2 * math.pi
            elif diff_theta < -math.pi:
                diff_theta += 2 * math.pi

            msg.angular.z = 6 * diff_theta
        else:
            # Target reached
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            self.call_catch_turtle_service(self.target_.name)
            self.target_ = None # Stop chasing until a new target is published

        self.cmd_vel_publisher_.publish(msg)

    def call_catch_turtle_service(self, turtle_name):
        client = self.create_client(CatchTurtle, "catch_turtle")
        while not client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for CatchTurtle service...")

        request = CatchTurtle.Request()
        request.name = turtle_name

        future = client.call_async(request)
        future.add_done_callback(self.callback_catch_turtle_response)

    def callback_catch_turtle_response(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(f"Successfully caught {self.target_.name}")
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = TurtleControllerNode()
    rclpy.spin(node)
    rclpy.shutdown()