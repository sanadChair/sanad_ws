#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import random
from turtlesim.srv import Spawn, Kill
from my_robot_interfaces.msg import Turtle, TurtleArray
from my_robot_interfaces.srv import CatchTurtle

class TurtleSpawnerNode(Node):
    def __init__(self):
        super().__init__("turtle_spawner")
        self.declare_parameter("spawn_frequency", 1.0)
        self.spawn_frequency_ = self.get_parameter("spawn_frequency").value
        self.turtle_counter_ = 0
        self.alive_turtles_ = []
        self.alive_turtles_publisher_ = self.create_publisher(
            TurtleArray, "alive_turtles", 10)
        self.publish_timer_ = self.create_timer(
            1.0, self.publish_alive_turtles)
        self.catch_turtle_service_ = self.create_service(
            CatchTurtle, "catch_turtle", self.callback_catch_turtle)

        self.spawn_new_turtle() # Spawn the first turtle

    def publish_alive_turtles(self):
        msg = TurtleArray()
        msg.turtles = self.alive_turtles_
        self.alive_turtles_publisher_.publish(msg)

    def spawn_new_turtle(self):
        self.turtle_counter_ += 1
        name = f"target_{self.turtle_counter_}"
        x = random.uniform(0.5, 10.5)
        y = random.uniform(0.5, 10.5)

        client = self.create_client(Spawn, "spawn")
        while not client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for Spawn service...")

        request = Spawn.Request()
        request.x = x
        request.y = y
        request.name = name

        future = client.call_async(request)
        future.add_done_callback(
            lambda future: self.callback_spawn(future, name, x, y))

    def callback_spawn(self, future, name, x, y):
        try:
            response = future.result()
            if response.name != "":
                new_turtle = Turtle()
                new_turtle.name = response.name
                new_turtle.x = x
                new_turtle.y = y
                self.alive_turtles_.append(new_turtle)
                self.get_logger().info(f"Turtle {response.name} spawned.")
                self.publish_alive_turtles()
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")

    def callback_catch_turtle(self, request, response):
        client = self.create_client(Kill, "kill")
        while not client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for Kill service...")

        kill_request = Kill.Request()
        kill_request.name = request.name
        future = client.call_async(kill_request)
        future.add_done_callback(
            lambda future: self.callback_kill(future, request.name))

        response.success = True
        return response

    def callback_kill(self, future, turtle_name):
        try:
            future.result()
            # Remove turtle from alive list
            self.alive_turtles_ = [
                t for t in self.alive_turtles_ if t.name != turtle_name]
            self.publish_alive_turtles()
            self.get_logger().info(f"Turtle {turtle_name} was caught and killed.")
            self.spawn_new_turtle() # Spawn a new one!
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = TurtleSpawnerNode()
    rclpy.spin(node)
    rclpy.shutdown()