#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import subprocess
import time

class SlamNode(Node):
    def __init__(self):
        super().__init__('slam_node')
        self.get_logger().info("Starting SLAM Node...")
        self.slam_process = None
        self.start_slam()

    def start_slam(self):
        """
        Start slam_toolbox in online_sync mode
        """
        self.get_logger().info("Launching slam_toolbox...")
        self.slam_process = subprocess.Popen([
            'ros2', 'launch', 'slam_toolbox', 'online_sync_launch.py',
            'use_sim_time:=true',
            'scan_topic:=/sanad/lidar_scan'
        ])
        self.get_logger().info("SLAM started. You can now drive the robot around.")

        # Wait some time for user to explore environment
        self.get_logger().info("Mapping in progress for 5 minutes...")
        time.sleep(5*60)  # Adjust based on map size

        # Save the map automatically
        self.save_map()

    def save_map(self):
        self.get_logger().info("Saving map...")
        map_name = 'room_map'
        save_proc = subprocess.run([
            'ros2', 'run', 'nav2_map_server', 'map_saver_cli',
            '-f', f'~/maps/{map_name}'
        ])
        if save_proc.returncode == 0:
            self.get_logger().info(f"Map saved successfully at ~/maps/{map_name}.yaml")
        else:
            self.get_logger().error("Map saving failed!")

        # Kill the slam_toolbox process
        if self.slam_process:
            self.get_logger().info("Stopping SLAM...")
            self.slam_process.terminate()
            self.slam_process.wait()
            self.get_logger().info("SLAM stopped.")


def main(args=None):
    rclpy.init(args=args)
    node = SlamNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
