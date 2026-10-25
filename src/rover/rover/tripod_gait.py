import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import math

class TripodGait(Node):

    def __init__(self):
        super().__init__('tripod_gait')

        self.publisher = self.create_publisher(
            JointTrajectory,
            '/hexapod_controller/joint_trajectory',
            10
        )

        self.timer = self.create_timer(1.0, self.publish_gait)
        self.phase = 0

        self.joint_names = [
            "NCCUP1_joint","NTHIGH1_joint","NFOOT1_joint",
            "NCCUP2_joint","NTHIGH2_joint","NFOOT2_joint",
            "NCCUP3_joint","NTHIGH3_joint","NFOOT3_joint",
            "NCCUP4_joint","NTHIGH4_joint","NFOOT4_joint",
            "NCCUP5_joint","NTHIGH5_joint","NFOOT5_joint",
            "NCCUP6_joint","NTHIGH6_joint","NFOOT6_joint"
        ]

    def publish_gait(self):
        msg = JointTrajectory()
        msg.joint_names = self.joint_names

        point = JointTrajectoryPoint()

        support = [0.0, -0.6, 1.2]
        swing   = [0.2, -0.2, 0.6]

        positions = []

        tripod_A = [1,3,5]
        tripod_B = [2,4,6]

        for leg in range(1,7):
            if (leg in tripod_A and self.phase == 0) or \
               (leg in tripod_B and self.phase == 1):
                positions.extend(swing)
            else:
                positions.extend(support)

        point.positions = positions
        point.time_from_start = Duration(sec=1)

        msg.points.append(point)

        self.publisher.publish(msg)

        self.phase = 1 - self.phase


def main(args=None):
    rclpy.init(args=args)
    node = TripodGait()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()