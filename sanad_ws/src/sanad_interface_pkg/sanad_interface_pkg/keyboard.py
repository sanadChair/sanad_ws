#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int8
import sys
import termios
import tty

class KeyboardTeleopNode(Node):
    def __init__(self):
        super().__init__('keyboard_teleop')
        
        # Publisher for movement commands
        self.pub = self.create_publisher(Int8, 'cmd_vel', 10)
        
        # Store terminal settings
        self.settings = termios.tcgetattr(sys.stdin)
        
        self.get_logger().info('Keyboard Teleop Node Started')
        self.print_instructions()
        
    def print_instructions(self):
        msg = """
        ---------------------------
        Keyboard Teleop Control
        ---------------------------
        Moving around:
           w    
        a  s  d
        
        w : forward (1)
        s : backward (2)
        a : left (3)
        d : right (4)
        
        Space : stop (0)
        q : quit
        ---------------------------
        """
        print(msg)
    
    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key
    
    def publish_command(self, command):
        msg = Int8()
        msg.data = command
        self.pub.publish(msg)
    
    def run(self):
        try:
            while True:
                key = self.get_key()
                
                if key == 'w':
                    # Move forward
                    self.publish_command(1)
                    self.get_logger().info('Command: Forward (1)')
                    
                elif key == 's':
                    # Move backward
                    self.publish_command(2)
                    self.get_logger().info('Command: Backward (2)')
                    
                elif key == 'a':
                    # Turn left
                    self.publish_command(3)
                    self.get_logger().info('Command: Left (3)')
                    
                elif key == 'd':
                    # Turn right
                    self.publish_command(4)
                    self.get_logger().info('Command: Right (4)')
                    
                elif key == ' ':
                    # Stop
                    self.publish_command(0)
                    self.get_logger().info('Command: Stop (0)')
                    
                elif key == 'q' or key == '\x03':  # q or Ctrl+C
                    # Quit
                    self.publish_command(0)
                    break
                    
        except Exception as e:
            self.get_logger().error(f'Error: {str(e)}')
            
        finally:
            # Stop robot and restore terminal
            self.publish_command(0)
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)


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