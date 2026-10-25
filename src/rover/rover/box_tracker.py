#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np


class BoxTracker(Node):

    def __init__(self):
        super().__init__('box_tracker')

        self.bridge = CvBridge()

        self.subscription = self.create_subscription(
            Image,
            '/camera/camera/image_raw',
            self.image_callback,
            10)

        self.cmd_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10)

        # Control gains
        self.k_ang = 0.0015
        self.k_lin = 0.0002

        # Desired box area (tune later)
        self.desired_area = 20000

        self.get_logger().info("Box Tracker Started")

    def image_callback(self, msg):

        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Red color mask (two ranges)
        lower_red1 = np.array([0, 120, 70])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([170, 120, 70])
        upper_red2 = np.array([180, 255, 255])

        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        mask = mask1 + mask2

        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        twist = Twist()

        if contours:
            largest = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(largest)

            if area > 500:

                x, y, w, h = cv2.boundingRect(largest)
                cx = x + w // 2
                image_center_x = frame.shape[1] // 2

                error_x = image_center_x - cx

                image_center_y = frame.shape[0] / 2
                object_center_y = y + h // 2
                error_y = image_center_y - object_center_y

                # Angular control
                twist.angular.z = self.k_ang * error_x

                # Forward control with proper stopping
                area_error = self.desired_area - area

                stop_area = self.desired_area * 0.95
                slow_area = self.desired_area * 0.8

                if abs(error_y) < 10:
                    twist.linear.x = 0.0
                else:
                    twist.linear.x = self.k_lin * error_y


                # Clamp speeds
                twist.linear.x = max(min(twist.linear.x, 0.2), -0.2)
                twist.angular.z = max(min(twist.angular.z, 0.5), -0.5)

        self.cmd_pub.publish(twist)


def main():
    rclpy.init()
    node = BoxTracker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
