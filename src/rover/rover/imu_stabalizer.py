#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Imu
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint


class BodyStabilizer(Node):

    def __init__(self):

        super().__init__("body_stabilizer")

        self.imu_sub = self.create_subscription(
            Imu,
            "/imu/data",
            self.imu_callback,
            10
        )

        self.traj_pub = self.create_publisher(
            JointTrajectory,
            "/hexapod_controller/joint_trajectory",
            10
        )

        self.roll = 0.0
        self.pitch = 0.0

        self.timer = self.create_timer(0.05, self.control_loop)

        self.joint_names = [
            "NCCUP1_joint","NTHIGH1_joint","NFOOT1_joint",
            "NCCUP2_joint","NTHIGH2_joint","NFOOT2_joint",
            "NCCUP3_joint","NTHIGH3_joint","NFOOT3_joint",
            "NCCUP4_joint","NTHIGH4_joint","NFOOT4_joint",
            "NCCUP5_joint","NTHIGH5_joint","NFOOT5_joint",
            "NCCUP6_joint","NTHIGH6_joint","NFOOT6_joint"
        ]

        self.get_logger().info("IMU Stabilizer Started")

    # ---------------- IMU ----------------

    def imu_callback(self,msg):

        x = msg.orientation.x
        y = msg.orientation.y
        z = msg.orientation.z
        w = msg.orientation.w

        self.roll = math.atan2(
            2*(w*x + y*z),
            1 - 2*(x*x + y*y)
        )

        self.pitch = math.asin(
            2*(w*y - z*x)
        )

    # ---------------- Control ----------------

    def control_loop(self):

        k = 0.1

        roll_correction  = -k * self.roll
        pitch_correction = -k * self.pitch

        # simple leg height offsets
        leg_offsets = [

            pitch_correction + roll_correction,
            pitch_correction - roll_correction,
            pitch_correction + roll_correction,

            -pitch_correction + roll_correction,
            -pitch_correction - roll_correction,
            -pitch_correction + roll_correction

        ]

        positions = []

        for i in range(6):

            j1 = 0.0
            j2 = 0.3 + leg_offsets[i]
            j3 = 0.6 - leg_offsets[i]

            positions.extend([j1,j2,j3])

        traj = JointTrajectory()
        traj.joint_names = self.joint_names

        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start.sec = 0
        point.time_from_start.nanosec = 50000000

        traj.points.append(point)

        self.traj_pub.publish(traj)


def main(args=None):

    rclpy.init(args=args)

    node = BodyStabilizer()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
