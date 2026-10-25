#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

JOINTS = [
    'base_link_Revolute-10',
    'arm_base_Revolute-11',
    'bicep_Revolute-12',
    'forearm_Revolute-13',
    'forearm_Revolute-14'
]

class ArmCommander(Node):

    def __init__(self):
        super().__init__('arm_terminal_commander')
        self.publisher = self.create_publisher(
            JointTrajectory,
            '/arm_controller/joint_trajectory',
            10
        )

    def send_trajectory(self, positions, duration=2):
        msg = JointTrajectory()
        msg.joint_names = JOINTS

        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start.sec = duration

        msg.points.append(point)
        self.publisher.publish(msg)
        self.get_logger().info(f"Sent: {positions}")


def main():
    rclpy.init()
    node = ArmCommander()

    print("\nEnter 5 joint angles in radians separated by space")
    print("Example: 0.0 0.5 -0.3 0.2 0.2\n")

    try:
        while rclpy.ok():
            user_input = input(">>> ")
            angles = list(map(float, user_input.split()))

            if len(angles) != 5:
                print("Need exactly 5 values.")
                continue

            node.send_trajectory(angles)

    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
