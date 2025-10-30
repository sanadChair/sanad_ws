#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO
import time


class PersonFollower(Node):
    def __init__(self):
        super().__init__("human_follower")

        # --- Setup ---
        self.bridge = CvBridge()
        self.model = YOLO("yolov8n.pt")  # Lightweight pretrained model
        self.image_sub = self.create_subscription(
            Image, "/camera/image_raw", self.image_callback, 10
        )
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)

        # --- Motion control variables ---
        self.last_seen_time = time.time()
        self.no_person_timeout = 2.0  # seconds before starting rotation search
        self.rotation_speed = -0.2  # rad/s while searching
        self.follow_distance_threshold = 0.5  # meters (approximation)
        self.prev_area = None  # For approximate distance estimation

        self.get_logger().info("Person Follower Node Started ✅")

    def rotate_in_place(self):
        """Rotate slowly to search for a person."""
        move_cmd = Twist()
        move_cmd.angular.z = self.rotation_speed
        move_cmd.linear.x = 0.0
        self.cmd_pub.publish(move_cmd)

    def stop_robot(self):
        """Stop robot movement."""
        stop_cmd = Twist()
        self.cmd_pub.publish(stop_cmd)

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        results = self.model(frame, verbose=False)
        h, w, _ = frame.shape
        cx_img = w // 2
        move_cmd = Twist()

        # Detect persons
        person_boxes = [box for box in results[0].boxes if int(box.cls[0]) == 0]

        if len(person_boxes) > 0:
            # --- Reset timer since a person is visible ---
            self.last_seen_time = time.time()

            # --- Pick closest (largest box area) ---
            largest = max(
                person_boxes,
                key=lambda b: (b.xyxy[0][2] - b.xyxy[0][0])
                * (b.xyxy[0][3] - b.xyxy[0][1]),
            )
            x1, y1, x2, y2 = largest.xyxy[0]
            cx = int((x1 + x2) / 2)
            cy = int((y1 + y2) / 2)
            area = (x2 - x1) * (y2 - y1)

            # --- Visual marker ---
            cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
            cv2.circle(frame, (cx, cy), 5, (0, 0, 255), -1)

            # --- Control logic ---
            error_x = cx - cx_img
            move_cmd.angular.z = float(error_x) / 600  # rotate to center person

            # Estimate distance based on bounding box area
            # (larger area = closer person)
            # You can adjust threshold depending on your camera FOV and distance
            too_close = area > 80000  # stop threshold (≈ 0.5 m)
            move_cmd.linear.x = -0.5 if not too_close else 0.0

            self.cmd_pub.publish(move_cmd)

        else:
            # --- No person detected ---
            if time.time() - self.last_seen_time > self.no_person_timeout:
                # Rotate to search
                self.rotate_in_place()
            else:
                # Stop briefly after losing sight
                self.stop_robot()

        # Show the processed frame
        cv2.imshow("Camera View", frame)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = PersonFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.stop_robot()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
