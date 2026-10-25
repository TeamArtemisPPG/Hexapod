#!/usr/bin/env python3
"""
odom_from_tf.py
Publishes a nav_msgs/Odometry message on /odom by reading TF.
When hardware wheel encoders are absent on a real hexapod, reads
map -> base_footprint (or odom -> base_footprint) from SLAM / TF tree
and computes linear/angular velocities dynamically so Nav2 costmaps
and controllers receive reliable dynamic motion feedback.
"""

import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
import tf2_ros


def yaw_from_quaternion(q):
    """Extract yaw angle from quaternion."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class OdomFromTF(Node):
    def __init__(self):
        super().__init__('odom_from_tf')

        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('rate', 20.0)

        self.base_frame = self.get_parameter('base_frame').get_parameter_value().string_value
        self.odom_frame = self.get_parameter('odom_frame').get_parameter_value().string_value
        rate = self.get_parameter('rate').get_parameter_value().double_value

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.pub = self.create_publisher(Odometry, '/odom', 10)
        self.timer = self.create_timer(1.0 / rate, self.publish_odom)

        self.last_x = None
        self.last_y = None
        self.last_yaw = None
        self.last_time = None

        self.get_logger().info(
            f"odom_from_tf started: base_frame='{self.base_frame}', odom_frame='{self.odom_frame}'"
        )

    def publish_odom(self):
        tf: TransformStamped = None
        for parent_frame in ['map', self.odom_frame]:
            try:
                tf = self.tf_buffer.lookup_transform(
                    parent_frame,
                    self.base_frame,
                    rclpy.time.Time()
                )
                break
            except (tf2_ros.LookupException,
                    tf2_ros.ConnectivityException,
                    tf2_ros.ExtrapolationException):
                continue

        if tf is None:
            return

        current_time = self.get_clock().now()
        stamp_sec = tf.header.stamp.sec + tf.header.stamp.nanosec * 1e-9

        x = tf.transform.translation.x
        y = tf.transform.translation.y
        z = tf.transform.translation.z
        q = tf.transform.rotation
        yaw = yaw_from_quaternion(q)

        vx = 0.0
        vy = 0.0
        wz = 0.0

        if self.last_time is not None:
            dt = stamp_sec - self.last_time
            if 0.001 < dt < 1.0:
                dx = x - self.last_x
                dy = y - self.last_y
                dyaw = yaw - self.last_yaw

                while dyaw > math.pi:
                    dyaw -= 2.0 * math.pi
                while dyaw < -math.pi:
                    dyaw += 2.0 * math.pi

                vx = dx / dt
                vy = dy / dt
                wz = dyaw / dt

        self.last_x = x
        self.last_y = y
        self.last_yaw = yaw
        self.last_time = stamp_sec

        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame

        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.position.z = z
        odom.pose.pose.orientation = q

        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.angular.z = wz

        # Covariance reflects lidar-SLAM uncertainty on a hexapod with no wheel encoders.
        # x/y: ~0.1 (10 cm), yaw: ~0.05 rad — realistic SLAM drift values.
        # Tight (1e-3) values caused Nav2 to over-trust poses and issue erratic replans.
        pose_cov = [0.10, 0.0, 0.0, 0.0, 0.0, 0.0,
                    0.0, 0.10, 0.0, 0.0, 0.0, 0.0,
                    0.0, 0.0,  1e6, 0.0, 0.0, 0.0,
                    0.0, 0.0,  0.0, 1e6, 0.0, 0.0,
                    0.0, 0.0,  0.0, 0.0, 1e6, 0.0,
                    0.0, 0.0,  0.0, 0.0, 0.0, 0.05]
        twist_cov = [0.05, 0.0, 0.0, 0.0, 0.0, 0.0,
                     0.0, 0.05, 0.0, 0.0, 0.0, 0.0,
                     0.0, 0.0,  1e6, 0.0, 0.0, 0.0,
                     0.0, 0.0,  0.0, 1e6, 0.0, 0.0,
                     0.0, 0.0,  0.0, 0.0, 1e6, 0.0,
                     0.0, 0.0,  0.0, 0.0, 0.0, 0.05]
        odom.pose.covariance = pose_cov
        odom.twist.covariance = twist_cov

        self.pub.publish(odom)


def main(args=None):
    rclpy.init(args=args)
    node = OdomFromTF()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
