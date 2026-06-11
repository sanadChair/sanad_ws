#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np

class DepthTestNode(Node):
    def __init__(self):
        super().__init__('depth_test_node')
        
        # Change this topic if your Astra driver uses a different namespace
        # Typical Astra Pro topic: '/camera/depth/image_raw'
        self.subscription = self.create_subscription(
            Image,
            '/camera/depth/image_raw',
            self.depth_callback,
            10
        )
        
        self.bridge = CvBridge()
        self.get_logger().info('Depth Test Node has been started. Waiting for depth images...')

    def depth_callback(self, msg):
        try:
            # OpenNI2 / Astra Pro depth images are typically 16-bit unsigned integers (OpenCV: cv::DataType<uint16_t>::type)
            # Encoding is usually '16UC1' representing depth in millimeters
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except CvBridgeError as e:
            self.get_logger().error(f'CvBridge Error: {e}')
            return

        # Get image dimensions (typically 640x480 for Astra Pro)
        height, width = cv_image.shape

        # Define a central bounding box window (e.g., 40x40 pixels) to calculate distance
        window_size = 40
        start_x = (width - window_size) // 2
        start_y = (height - window_size) // 2
        
        # Crop the center region
        center_roi = cv_image[start_y:start_y+window_size, start_x:start_x+window_size]

        # Filter out invalid readings (0 or NaN/Inf)
        # Astra Pro returns 0 for pixels that are too close (<0.6m), too far, or in an IR shadow
        valid_depths = center_roi[center_roi > 0]

        if valid_depths.size > 0:
            # Calculate the median to minimize the effect of pixel jitter or edge noise
            median_depth_mm = np.median(valid_depths)
            median_depth_m = median_depth_mm / 1000.0  # Convert to meters
            
            self.get_logger().info(
                f"Calculated Center Depth: {median_depth_m:.3f} m ({median_depth_mm:.1f} mm) | "
                f"Valid Pixels in Window: {valid_depths.size}/{window_size*window_size}"
            )
        else:
            self.get_logger().warn(
                "No valid depth data in the center window! Ensure target is > 0.6m away and not in direct sunlight."
            )

def main(args=None):
    rclpy.init(args=args)
    node = DepthTestNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()