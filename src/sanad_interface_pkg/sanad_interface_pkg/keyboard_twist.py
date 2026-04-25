#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import termios
import tty

class KeyboardTeleopNode(Node):
    def __init__(self):
        super().__init__('keyboard_teleop_node')
        
        # Publisher for velocity commands
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Movement parameters
        self.linear_speed = 0.5  # m/s
        self.angular_speed = 1.0  # rad/s
        self.linear_increment = 0.1
        self.angular_increment = 0.2
        
        self.get_logger().info('Keyboard Teleop Node Started')
        self.print_instructions()
        
    def print_instructions(self):
        msg = """
        ================================
        Keyboard Teleop Control
        ================================
        Moving around:
           w
        a  s  d
        
        w/s : increase/decrease linear speed
        a/d : increase/decrease angular speed
        
        i/k : move forward/backward
        j/l : turn left/right
        
        space : stop immediately
        q : quit
        
        Current speeds:
        - Linear: {:.2f} m/s
        - Angular: {:.2f} rad/s
        ================================
        """.format(self.linear_speed, self.angular_speed)
        print(msg)
    
    def get_key(self):
        """Get a single keypress from the terminal"""
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            key = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return key
    
    def publish_velocity(self, linear, angular):
        """Publish velocity command"""
        twist = Twist()
        twist.linear.x = linear
        twist.angular.z = angular
        self.publisher_.publish(twist)
    
    def run(self):
        """Main control loop"""
        try:
            while rclpy.ok():
                key = self.get_key()
                
                if key == 'q' or key == '\x03':  # q or Ctrl+C
                    self.get_logger().info('Shutting down...')
                    self.publish_velocity(0.0, 0.0)
                    break
                
                elif key == 'i':  # Forward
                    self.publish_velocity(self.linear_speed, 0.0)
                    self.get_logger().info('Moving forward')
                
                elif key == 'k':  # Backward
                    self.publish_velocity(-self.linear_speed, 0.0)
                    self.get_logger().info('Moving backward')
                
                elif key == 'j':  # Turn left
                    self.publish_velocity(0.0, self.angular_speed)
                    self.get_logger().info('Turning left')
                
                elif key == 'l':  # Turn right
                    self.publish_velocity(0.0, -self.angular_speed)
                    self.get_logger().info('Turning right')
                
                elif key == 'w':  # Increase linear speed
                    self.linear_speed += self.linear_increment
                    self.get_logger().info(f'Linear speed: {self.linear_speed:.2f} m/s')
                
                elif key == 's':  # Decrease linear speed
                    self.linear_speed = max(0.0, self.linear_speed - self.linear_increment)
                    self.get_logger().info(f'Linear speed: {self.linear_speed:.2f} m/s')
                
                elif key == 'a':  # Increase angular speed
                    self.angular_speed += self.angular_increment
                    self.get_logger().info(f'Angular speed: {self.angular_speed:.2f} rad/s')
                
                elif key == 'd':  # Decrease angular speed
                    self.angular_speed = max(0.0, self.angular_speed - self.angular_increment)
                    self.get_logger().info(f'Angular speed: {self.angular_speed:.2f} rad/s')
                
                elif key == ' ':  # Stop
                    self.publish_velocity(0.0, 0.0)
                    self.get_logger().info('Stopped')
                
        except Exception as e:
            self.get_logger().error(f'Error: {str(e)}')
        finally:
            # Make sure robot stops when exiting
            self.publish_velocity(0.0, 0.0)


def main(args=None):
    rclpy.init(args=args)
    
    node = KeyboardTeleopNode()
    
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()