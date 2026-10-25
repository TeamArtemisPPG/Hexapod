#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import math

JOINTS = [
    'base_link_Revolute-10',
    'arm_base_Revolute-11',
    'bicep_Revolute-12',
    'forearm_Revolute-13',
    'forearm_Revolute-14'
]

# Adjustable link lengths (meters)
L1 = 0.14
L2 = 0.20

class ArmIKCommander(Node):

    def __init__(self):
        super().__init__('arm_ik_commander')
        self.publisher = self.create_publisher(
            JointTrajectory,
            '/arm_controller/joint_trajectory',
            10
        )

    def compute_ik(self, x, y, z):

        L1 = 0.14
        L2 = 0.20

        # Base rotation
        theta1 = math.atan2(y, x)

        # Planar reduction
        r = math.sqrt(x*x + y*y)
        d = math.sqrt(r*r + z*z)

        # Reach check
        if d > (L1 + L2) or d < abs(L1 - L2):
            raise ValueError("Target unreachable")

        # Law of cosines
        D = math.atan2(z, r)
        E = math.acos((L1*L1 + d*d - L2*L2) / (2 * L1 * d))
        theta2 = D + E

        theta3 = math.acos((L1*L1 + L2*L2 - d*d) / (2 * L1 * L2))

        theta1 = theta1 + math.pi/2   # add 90 degrees
        theta2 = -theta2              # invert shoulder

        return [theta1, theta2, theta3, 0.0, 0.0]

    def send_trajectory(self, joint_positions):

        msg = JointTrajectory()
        msg.joint_names = JOINTS

        point = JointTrajectoryPoint()
        point.positions = joint_positions
        point.time_from_start.sec = 2

        msg.points.append(point)

        self.publisher.publish(msg)
        self.get_logger().info(f"Moving to: {joint_positions}")


def main():
    rclpy.init()
    node = ArmIKCommander()

    print("\nEnter target X Y Z (meters)")
    print("Example: 0.3 0.1 0.2\n")

    try:
        while rclpy.ok():
            user_input = input(">>> ")
            x, y, z = map(float, user_input.split())

            # Compute IK
            joint_angles = node.compute_ik(x, y, z)

            # Convert to degrees
            deg_angles = [math.degrees(a) for a in joint_angles]

            # Print radians
            print("\nJoint Angles (Radians):")
            print(f"  Base        : {joint_angles[0]:.4f}")
            print(f"  Shoulder    : {joint_angles[1]:.4f}")
            print(f"  Elbow       : {joint_angles[2]:.4f}")
            print(f"  Claw1       : {joint_angles[3]:.4f}")
            print(f"  Claw2       : {joint_angles[4]:.4f}")

            # Print degrees
            print("\nJoint Angles (Degrees):")
            print(f"  Base        : {deg_angles[0]:.2f}°")
            print(f"  Shoulder    : {deg_angles[1]:.2f}°")
            print(f"  Elbow       : {deg_angles[2]:.2f}°")
            print(f"  Claw1       : {deg_angles[3]:.2f}°")
            print(f"  Claw2       : {deg_angles[4]:.2f}°\n")

            # Send to controller
            node.send_trajectory(joint_angles)

    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
