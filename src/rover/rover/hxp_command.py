#!/usr/bin/env python3

import math
import signal
import sys
import threading
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint


class IKController(Node):

    def __init__(self):
        super().__init__('ik_controller')

        self.running = True

        # -------- Publishers --------
        self.publisher = self.create_publisher(
            JointTrajectory,
            '/hexapod_controller/joint_trajectory',
            10
        )

        self.js_pub = self.create_publisher(
            JointState,
            '/joint_states',
            10
        )

        # -------- Subscriber: /cmd_vel from Nav2 / teleop --------
        self.cmd_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )
        self.last_cmd_time = None
        self.cmd_timeout = 0.6

        # -------- Timing --------
        self.dt = 0.125
        self.timer = self.create_timer(self.dt, self.update)

        self.step_time = 0.5
        self.step_timer = 0.0
        self.step_index = 0
        self.num_steps = 4   # IMPORTANT: now 4 steps

        # -------- Command (default 's' stand still; moves only when cmd_vel received or manual command) --------
        self.command = 's'

        # -------- Joint order --------
        self.joint_names = [
            "NCCUP1_joint","NTHIGH1_joint","NFOOT1_joint",
            "NCCUP2_joint","NTHIGH2_joint","NFOOT2_joint",
            "NCCUP3_joint","NTHIGH3_joint","NFOOT3_joint",
            "NCCUP4_joint","NTHIGH4_joint","NFOOT4_joint",
            "NCCUP5_joint","NTHIGH5_joint","NFOOT5_joint",
            "NCCUP6_joint","NTHIGH6_joint","NFOOT6_joint"
        ]

        self.positions = [0.0]*18

        # -------- FORWARD GAIT --------
        self.forward_gait = {
            1: [(77.8,77.8,150), (95.8,59.8,150), (77.8,77.8,100), (59.8,95.8,150)],
            2: [(55,80,150), (80,80,150), (55,80,100), (30,80,150)],
            3: [(80,55,150), (80,80,150), (80,55,100), (80,30,150)],
            4: [(77.8,77.8,150), (59.8,95.8,150), (77.8,77.8,100), (95.8,59.8,150)],
            5: [(55,80,150), (30,80,150), (55,80,100), (80,80,150)],
            6: [(80,55,150), (80,30,150), (80,55,100), (80,80,150)],
        }

        # -------- RIGHT TURN GAIT --------
        self.right_gait = {
            1: [(77.8,77.8,150), (59.3,95.0,150), (77.8,77.8,100), (95.0,59.3,150)],
            2: [(77.8,77.8,150), (59.3,95.0,150), (77.8,77.8,100), (95.0,59.3,150)],
            3: [(77.8,77.8,150), (59.3,95.0,150), (77.8,77.8,100), (95.0,59.3,150)],
            4: [(77.8,77.8,150), (59.3,95.0,150), (77.8,77.8,100), (95.0,59.3,150)],
            5: [(77.8,77.8,150), (59.3,95.0,150), (77.8,77.8,100), (95.0,59.3,150)],
            6: [(77.8,77.8,150), (59.3,95.0,150), (77.8,77.8,100), (95.0,59.3,150)],
        }

        # -------- INITIAL / STAND POSE --------
        self.initial_pose = {
            1: (77.8, 77.8, 150),
            2: (55.0, 80.0, 150),
            3: (80.0, 55.0, 150),
            4: (77.8, 77.8, 150),
            5: (55.0, 80.0, 150),
            6: (80.0, 55.0, 150),
        }

        # -------- Tripod groups --------
        self.tripod_A = [1,3,5]
        self.tripod_B = [2,4,6]

        # -------- Start input thread --------
        threading.Thread(target=self.keyboard_input, daemon=True).start()

        self.get_logger().info("Controller started (f/b/l/r to move, s to stand/initial, q to quit)")

    # -------- Cmd Vel Callback (Nav2) --------
    def cmd_vel_callback(self, msg: Twist):
        self.last_cmd_time = self.get_clock().now()
        vx = msg.linear.x
        wz = msg.angular.z

        lin_threshold = 0.03
        ang_threshold = 0.08

        prev_cmd = self.command

        # If angular velocity dominates, prioritize turning; otherwise drive forward/backward
        if abs(wz) >= ang_threshold and (abs(vx) < lin_threshold or abs(wz) > 1.2 * abs(vx)):
            self.command = 'l' if wz > 0.0 else 'r'
        elif abs(vx) >= lin_threshold:
            self.command = 'f' if vx > 0.0 else 'b'
        elif abs(wz) >= ang_threshold:
            self.command = 'l' if wz > 0.0 else 'r'
        else:
            self.command = 's'

        if self.command != prev_cmd:
            self.get_logger().info(f"Nav2 gait command: {prev_cmd} -> {self.command} (vx={vx:.2f}, wz={wz:.2f})")

    # -------- Keyboard Input --------
    def keyboard_input(self):
        # Do not block or throw EOFError if running non-interactively via ros2 launch
        if not sys.stdin or not sys.stdin.isatty():
            return

        while self.running:
            try:
                cmd = input("Enter command (f/b/l/r/s, q to quit): ").strip().lower()
                if cmd in ['f', 'b', 'l', 'r', 's']:
                    self.command = cmd
                    self.last_cmd_time = None
                    print(f"Switched to: {cmd}")
                elif cmd in ['q', 'quit']:
                    print("Stopping program and returning to initial position...")
                    self.running = False
                    break
            except (EOFError, KeyboardInterrupt):
                break

    # -------- Gait Selection --------
    def get_gait(self):

        if self.command == 'f':
            return self.forward_gait

        elif self.command == 'b':
            return {leg: list(reversed(points)) for leg, points in self.forward_gait.items()}

        elif self.command == 'r':
            return self.right_gait

        elif self.command == 'l':
            return {leg: list(reversed(points)) for leg, points in self.right_gait.items()}

        return self.forward_gait

    # -------- Inverse Kinematics --------
    def inverse_kinematics(self, x, y, z):

        J1L = 35.0
        J2L = 75.0
        J3L = 160.0

        J1 = math.atan2(x, y)

        H = math.sqrt(x*x + y*y)
        L = math.sqrt(z*z + (H - J1L)*(H - J1L))

        def clamp(v):
            return max(-1.0, min(1.0, v))

        J3 = 1.57 - math.acos(clamp((J3L*J3L + J2L*J2L - L*L)/(2*J2L*J3L)))

        B = math.acos(clamp((L*L + J2L*J2L - J3L*J3L)/(2*L*J2L)))
        A = math.atan2(z, H - J1L)

        J2 = (B - A)

        return J1, J2, J3

    # -------- Gait update --------
    def update_gait(self):

        self.step_timer += self.dt

        if self.step_timer >= self.step_time:
            self.step_timer = 0.0
            self.step_index = (self.step_index + 1) % self.num_steps

    # -------- Main Loop --------
    def update(self):

        # If controlled by /cmd_vel, stop if commands time out
        if self.last_cmd_time is not None:
            elapsed = (self.get_clock().now() - self.last_cmd_time).nanoseconds * 1e-9
            if elapsed > self.cmd_timeout:
                self.command = 's'

        if self.command == 's':
            # Hold initial standing position
            for leg_id in range(1, 7):
                x, y, z = self.initial_pose[leg_id]
                q1, q2, q3 = self.inverse_kinematics(x, y, z)
                if leg_id == 5:
                    q2 = -q2
                    q3 = -q3
                base = (leg_id - 1) * 3
                self.positions[base]   = q1
                self.positions[base+1] = q2
                self.positions[base+2] = q3
            self.publish_trajectory()
            self.publish_joint_states()
            return

        self.update_gait()
        gait = self.get_gait()

        for leg_id in range(1,7):

            if leg_id in self.tripod_A:
                idx = self.step_index
            else:
                idx = (self.step_index + self.num_steps//2) % self.num_steps
            
            x, y, z = gait[leg_id][idx]

            q1, q2, q3 = self.inverse_kinematics(x, y, z)
            if leg_id == 5:
                q2 = -q2
                q3 = -q3

            base = (leg_id-1)*3
            self.positions[base]   = q1
            self.positions[base+1] = q2
            self.positions[base+2] = q3

        self.publish_trajectory()
        self.publish_joint_states()

    # -------- Publish Trajectory --------
    def publish_trajectory(self):

        msg = JointTrajectory()
        msg.joint_names = self.joint_names

        point = JointTrajectoryPoint()
        point.positions = self.positions
        point.time_from_start.sec = 1

        msg.points.append(point)
        self.publisher.publish(msg)

    # -------- Publish Joint States (RViz) --------
    def publish_joint_states(self):

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names + ["wheel_front_right_joint", "wheel_front_left_joint"]
        msg.position = self.positions + [0.0, 0.0]

        self.js_pub.publish(msg)

    # -------- Reset to Initial Position --------
    def reset_to_initial_pose(self):
        self.get_logger().info("Setting robot to initial position...")

        if hasattr(self, 'timer') and self.timer is not None:
            self.timer.cancel()

        for leg_id in range(1, 7):
            x, y, z = self.initial_pose[leg_id]
            q1, q2, q3 = self.inverse_kinematics(x, y, z)
            if leg_id == 5:
                q2=-q2
                q3=-q3
            base = (leg_id - 1) * 3
            self.positions[base]   = q1
            self.positions[base+1] = q2
            self.positions[base+2] = q3

        if rclpy.ok():
            for _ in range(5):
                self.publish_trajectory()
                self.publish_joint_states()
                try:
                    rclpy.spin_once(self, timeout_sec=0.1)
                except Exception:
                    pass
        self.get_logger().info("Robot is in initial position.")


def main(args=None):

    rclpy.init(args=args)
    node = IKController()

    def sig_handler(sig, frame):
        node.get_logger().info("Stop signal received, shutting down...")
        node.running = False

    signal.signal(signal.SIGINT, sig_handler)
    signal.signal(signal.SIGTERM, sig_handler)

    try:
        while node.running and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.reset_to_initial_pose()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()