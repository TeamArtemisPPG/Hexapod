#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np


class ObjectDetector(Node):

    def __init__(self):
        super().__init__('object_detector')

        self.bridge = CvBridge()

        self.subscription = self.create_subscription(
            Image,
            '/camera/camera/image_raw',
            self.image_callback,
            10
        )

        self.get_logger().info("Object Detector Started")

    def image_callback(self, msg):

        # Convert ROS image → OpenCV image
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Convert to HSV
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Define red color range
        lower_red1 = np.array([0, 120, 70])
        upper_red1 = np.array([10, 255, 255])

        lower_red2 = np.array([170, 120, 70])
        upper_red2 = np.array([180, 255, 255])

        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)

        mask = mask1 + mask2

        # Find contours
        contours, _ = cv2.findContours(
            mask,
            cv2.RETR_TREE,
            cv2.CHAIN_APPROX_SIMPLE
        )

        for cnt in contours:
            area = cv2.contourArea(cnt)

            if area > 500:  # filter noise

                x, y, w, h = cv2.boundingRect(cnt)

                # Draw bounding box
                cv2.rectangle(frame,
                              (x, y),
                              (x + w, y + h),
                              (0, 255, 0),
                              2)

                # Center point
                cx = x + w // 2
                cy = y + h // 2

                cv2.circle(frame, (cx, cy), 5, (255, 0, 0), -1)

                self.get_logger().info(
                    f"Detected Object at pixel: ({cx}, {cy})"
                )

        cv2.imshow("Detection", frame)
        cv2.waitKey(1)


def main():
    rclpy.init()
    node = ObjectDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
