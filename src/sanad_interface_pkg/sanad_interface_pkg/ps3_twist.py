#!/usr/bin/env python3
import rclpy
import pygame
from rclpy.node import Node
from geometry_msgs.msg import Twist

# init pygame joystick
pygame.joystick.init()
pygame.init()


class PS3TeolepNode(Node):
    def __init__(self):
        super().__init__("ps3_twist")
        self.get_logger().info("PS3 node is started!")
        self._publisher = self.create_publisher(Twist, "cmd_vel", 10)
        self.twist = Twist()

    def updateTwist(self, axis, value):
        print(axis, value)
        if axis == 1:  # linear
            self.twist.linear.x = value * 1.5

        if axis == 0:  # angular
            self.twist.angular.z = value
        self._publisher.publish(self.twist)


def main(args=None):
    running = True
    rclpy.init(args=args)
    node = PS3TeolepNode()
    joystick = [pygame.joystick.Joystick(x) for x in range(pygame.joystick.get_count())]
    print(joystick)
    node.get_logger().info(str(joystick))
    while running:
        for event in pygame.event.get():
            node.get_logger().info(str(event))
            if event.type == pygame.JOYAXISMOTION:
                node.get_logger().info(
                    str(f" axis = {event.axis}, Value = {event.value}")
                )
                node.updateTwist(event.axis, event.value)
            elif event.type == pygame.JOYBUTTONUP and event.button == 11:
                node.get_logger().info("PS3 Control is Going To Bed")
                node.destroy_node()
                rclpy.shutdown()
                running = False


if __name__ == "__main__":
    main()
