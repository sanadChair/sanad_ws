#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import threading

class ConsoleInputNode(Node):
    def __init__(self):
        super().__init__('console_input_node')
        
        self.publisher_ = self.create_publisher(String, '/robot/voice_command', 10)
        self.subscription = self.create_subscription(
            String,
            '/robot/speak',
            self.speak_callback,
            10
        )
        
        self.get_logger().info("🔵 Console Input Node Ready. Type your commands below!")

    def speak_callback(self, msg):
        """Prints responses received from the TTS speak topic to the console."""
        print(f"\n🗣️ VOICE REPLY: {msg.data}")
        print("\n🔵 Command: ", end="", flush=True)

    def input_loop(self):
        """Reads stdin from the user and publishes it as a voice command."""
        print("\n🔵 Command: ", end="", flush=True)
        while rclpy.ok():
            try:
                user_input = input()
                if user_input.strip():
                    msg = String()
                    msg.data = user_input.strip()
                    self.publisher_.publish(msg)
            except EOFError:
                break
            except KeyboardInterrupt:
                break

def main(args=None):
    rclpy.init(args=args)
    node = ConsoleInputNode()
    
    # Spin in a background thread to handle subscriptions asynchronously
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()
    
    try:
        node.input_loop()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        spin_thread.join()

if __name__ == '__main__':
    main()
