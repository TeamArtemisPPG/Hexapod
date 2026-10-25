#!/usr/bin/env python3
"""
hexapod_nav_controller.py
Bridges Nav2 / cmd_vel commands to hexapod inverse kinematics and walking gaits.
- Subscribes to /cmd_vel (geometry_msgs/msg/Twist)
- Automatically switches between Forward, Backward, Turn Left, Turn Right, and Stand
- Solves Inverse Kinematics for 6 legs (18 DOF)
- Publishes JointTrajectory on /hexapod_controller/joint_trajectory (for ESP32 hardware bridge)
- Publishes JointState on /joint_states (for robot_state_publisher and RViz visualization)
"""

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState


class HexapodNavController(Node):

    def __init__(self):
        super().__init__('hexapod_nav_controller')

        # Parameters
        self.declare_parameter('cmd_timeout', 0.6)
        self.declare_parameter('step_time', 0.45)
        self.declare_parameter('lin_threshold', 0.03)
        self.declare_parameter('ang_threshold', 0.08)

        self.cmd_timeout = self.get_parameter('cmd_timeout').get_parameter_value().double_value
        self.step_time = self.get_parameter('step_time').get_parameter_value().double_value
        self.lin_threshold = self.get_parameter('lin_threshold').get_parameter_value().double_value
        self.ang_threshold = self.get_parameter('ang_threshold').get_parameter_value().double_value

        # Publishers
        self.traj_pub = self.create_publisher(
            JointTrajectory,
            '/hexapod_controller/joint_trajectory',
            10
        )
        self.js_pub = self.create_publisher(
            JointState,
            '/joint_states',
            10
        )

        # Subscriber: cmd_vel from Nav2 or teleop
        self.cmd_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

        # Timing
        self.dt = 0.1  # 10 Hz control loop
        self.timer = self.create_timer(self.dt, self.update)

        self.target_vx = 0.0
        self.target_wz = 0.0
        self.last_cmd_time = self.get_clock().now()

        self.step_timer = 0.0
        self.step_index = 0
        self.num_steps = 4
        self.current_state = 's'  # 's'=stand, 'f'=forward, 'b'=back, 'l'=left, 'r'=right

        # Joint Names
        self.joint_names = [
            "NCCUP1_joint", "NTHIGH1_joint", "NFOOT1_joint",
            "NCCUP2_joint", "NTHIGH2_joint", "NFOOT2_joint",
            "NCCUP3_joint", "NTHIGH3_joint", "NFOOT3_joint",
            "NCCUP4_joint", "NTHIGH4_joint", "NFOOT4_joint",
            "NCCUP5_joint", "NTHIGH5_joint", "NFOOT5_joint",
            "NCCUP6_joint", "NTHIGH6_joint", "NFOOT6_joint"
        ]
        self.positions = [0.0] * 18

        # Gait Definition (coordinates in mm)
        self.forward_gait = {
            1: [(77.8, 77.8, 120), (95.8, 59.8, 120), (77.8, 77.8, 100), (59.8, 95.8, 120)],
            6: [(55.0, 80.0, 120), (80.0, 80.0, 120), (55.0, 80.0, 100), (30.0, 80.0, 120)],
            5: [(80.0, 55.0, 120), (80.0, 80.0, 120), (80.0, 55.0, 100), (80.0, 30.0, 120)],
            4: [(77.8, 77.8, 120), (59.8, 95.8, 120), (77.8, 77.8, 100), (95.8, 59.8, 120)],
            3: [(55.0, 80.0, 120), (30.0, 80.0, 120), (55.0, 80.0, 100), (80.0, 80.0, 120)],
            2: [(80.0, 55.0, 120), (80.0, 30.0, 120), (80.0, 55.0, 100), (80.0, 80.0, 120)],
        }

        self.right_gait = {
            1: [(77.8, 77.8, 120), (59.3, 95.0, 120), (77.8, 77.8, 100), (95.0, 59.3, 120)],
            2: [(77.8, 77.8, 120), (59.3, 95.0, 120), (77.8, 77.8, 100), (95.0, 59.3, 120)],
            3: [(77.8, 77.8, 120), (59.3, 95.0, 120), (77.8, 77.8, 100), (95.0, 59.3, 120)],
            4: [(77.8, 77.8, 120), (59.3, 95.0, 120), (77.8, 77.8, 100), (95.0, 59.3, 120)],
            5: [(77.8, 77.8, 120), (59.3, 95.0, 120), (77.8, 77.8, 100), (95.0, 59.3, 120)],
            6: [(77.8, 77.8, 120), (59.3, 95.0, 120), (77.8, 77.8, 100), (95.0, 59.3, 120)],
        }

        self.stand_pose = {
            1: (77.8, 77.8, 120.0),
            2: (80.0, 55.0, 120.0),
            3: (55.0, 80.0, 120.0),
            4: (77.8, 77.8, 120.0),
            5: (80.0, 55.0, 120.0),
            6: (55.0, 80.0, 120.0),
        }

        self.tripod_A = [1, 3, 5]
        self.tripod_B = [2, 4, 6]

        self.get_logger().info("HexapodNavController initialized (Listening to /cmd_vel)")

    def cmd_vel_callback(self, msg: Twist):
        self.target_vx = msg.linear.x
        self.target_wz = msg.angular.z
        self.last_cmd_time = self.get_clock().now()

    def inverse_kinematics(self, x, y, z):
        J1L = 35.0
        J2L = 75.0
        J3L = 160.0

        J1 = math.atan2(x, y)
        H = math.sqrt(x * x + y * y)
        L = math.sqrt(z * z + (H - J1L) * (H - J1L))

        def clamp(v):
            return max(-1.0, min(1.0, v))

        J3 = 1.57 - math.acos(clamp((J3L * J3L + J2L * J2L - L * L) / (2 * J2L * J3L)))
        B = math.acos(clamp((L * L + J2L * J2L - J3L * J3L) / (2 * L * J2L)))
        A = math.atan2(z, H - J1L)
        J2 = B - A

        return J1, J2, J3

    def update(self):
        # Check command freshness
        elapsed = (self.get_clock().now() - self.last_cmd_time).nanoseconds * 1e-9
        if elapsed > self.cmd_timeout:
            self.target_vx = 0.0
            self.target_wz = 0.0

        # Determine motion state
        new_state = 's'
        if abs(self.target_vx) >= self.lin_threshold:
            new_state = 'f' if self.target_vx > 0.0 else 'b'
        elif abs(self.target_wz) >= self.ang_threshold:
            new_state = 'l' if self.target_wz > 0.0 else 'r'

        if new_state != self.current_state:
            self.get_logger().info(f"State transition: {self.current_state} -> {new_state} (vx={self.target_vx:.2f}, wz={self.target_wz:.2f})")
            self.current_state = new_state
            self.step_timer = 0.0
            self.step_index = 0

        # Compute leg targets
        if self.current_state == 's':
            # Standing still: all legs planted in neutral position
            for leg_id in range(1, 7):
                x, y, z = self.stand_pose[leg_id]
                q1, q2, q3 = self.inverse_kinematics(x, y, z)
                if leg_id == 5:
                    q2 = -q2
                    q3 = -q3
                base = (leg_id - 1) * 3
                self.positions[base] = q1
                self.positions[base + 1] = q2
                self.positions[base + 2] = q3
        else:
            # Step through gait cycle
            self.step_timer += self.dt
            if self.step_timer >= self.step_time:
                self.step_timer = 0.0
                self.step_index = (self.step_index + 1) % self.num_steps

            if self.current_state == 'f':
                gait = self.forward_gait
            elif self.current_state == 'b':
                gait = {leg: list(reversed(pts)) for leg, pts in self.forward_gait.items()}
            elif self.current_state == 'r':
                gait = self.right_gait
            elif self.current_state == 'l':
                gait = {leg: list(reversed(pts)) for leg, pts in self.right_gait.items()}
            else:
                gait = self.forward_gait

            for leg_id in range(1, 7):
                if leg_id in self.tripod_A:
                    idx = self.step_index
                else:
                    idx = (self.step_index + self.num_steps // 2) % self.num_steps

                x, y, z = gait[leg_id][idx]
                q1, q2, q3 = self.inverse_kinematics(x, y, z)
                if leg_id == 5:
                    q2 = -q2
                    q3 = -q3
                base = (leg_id - 1) * 3
                self.positions[base] = q1
                self.positions[base + 1] = q2
                self.positions[base + 2] = q3

        self.publish_trajectory()
        self.publish_joint_states()

    def publish_trajectory(self):
        msg = JointTrajectory()
        msg.joint_names = self.joint_names
        point = JointTrajectoryPoint()
        point.positions = self.positions
        point.time_from_start.sec = 1
        msg.points.append(point)
        self.traj_pub.publish(msg)

    def publish_joint_states(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = self.positions
        self.js_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = HexapodNavController()
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
