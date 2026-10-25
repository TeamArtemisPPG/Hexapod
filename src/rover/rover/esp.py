#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory

import serial
import math


class ESPBridge(Node):

    def __init__(self):
        super().__init__("esp_bridge")

        # -------- SERIAL PARAMETERS --------
        self.declare_parameter("port", "/dev/ttyUSB1")
        self.declare_parameter("baud", 115200)

        self.port = self.get_parameter("port").get_parameter_value().string_value
        self.baud = self.get_parameter("baud").get_parameter_value().integer_value

        self.connect_serial()

        # -------- SUBSCRIBE --------
        self.sub = self.create_subscription(
            JointTrajectory,
            "/hexapod_controller/joint_trajectory",
            self.callback,
            10
        )

        self.get_logger().info("ESP Bridge (FINAL - RAW ANGLES) Started")

    # -------- SERIAL CONNECT --------
    def connect_serial(self):
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=1)
            self.get_logger().info(f"Connected to {self.port}")
        except Exception as e:
            self.get_logger().error(f"Serial connection failed: {e}")
            self.ser = None

    # -------- SAFE WRITE --------
    def send_data(self, data):
        if self.ser is None:
            self.connect_serial()
            return

        try:
            self.ser.write(data.encode())
        except Exception as e:
            self.get_logger().warn(f"Serial error: {e}")
            try:
                self.ser.close()
            except:
                pass
            self.ser = None

    # -------- CALLBACK --------
    def callback(self, msg):

        if not msg.points:
            return

        positions = msg.points[0].positions

        # -------- EXPECT EXACTLY 18 --------
        if len(positions) != 18:
            self.get_logger().warn(f"Expected 18 joints, got {len(positions)}")
            return

        # -------- RAD → DEG ONLY --------
        #angles_deg = [round(math.degrees(p), 2) for p in positions]
        angles_deg = []

        # Calibrated neutral J1 atan2(x, y) angles from hxp_command gaits:
        # Legs 1 & 4: 45.0 deg | Legs 2 & 5: ~34.51 deg | Legs 3 & 6: ~55.49 deg
        j1_neutral = {
            1: 45.0,
            2: math.degrees(math.atan2(55.0, 80.0)),
            3: math.degrees(math.atan2(80.0, 55.0)),
            4: 45.0,
            5: math.degrees(math.atan2(55.0, 80.0)),
            6: math.degrees(math.atan2(80.0, 55.0)),
        }

        for i in range(18):
            angle_deg = math.degrees(positions[i])
            leg_id = (i // 3) + 1
            joint_type = i % 3

            if joint_type == 0:       # J1 (Coxa) — center precisely at 90 deg neutral
                servo_deg = 90.0 + (angle_deg - j1_neutral[leg_id])
            else:                     # J2, J3
                servo_deg = 90.0 + angle_deg

            # safety clamp to 0 - 180 degrees
            servo_deg = max(0.0, min(180.0, servo_deg))
            angles_deg.append(round(servo_deg, 2))

        # -------- FORMAT --------
        data = " ".join(map(str, angles_deg)) + "\n"

        # -------- SEND --------
        self.send_data(data)

        # Debug
        self.get_logger().info(f"Sent: {data.strip()}")


def main(args=None):
    rclpy.init(args=args)

    node = ESPBridge()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
# #!/usr/bin/env python3

# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import JointState

# import serial
# import math


# class JointStateToESP(Node):

#     def __init__(self):
#         super().__init__("jointstate_to_esp")

#         # -------- SERIAL --------
#         self.port = "/dev/ttyUSB0"
#         self.baud = 115200
#         self.connect_serial()

#         # -------- SUBSCRIBE --------
#         self.sub = self.create_subscription(
#             JointState,
#             "/joint_states",
#             self.callback,
#             10
#         )

#         self.get_logger().info("JointState → ESP Bridge (FINAL) Started")

#         # -------- REQUIRED ORDER (VERY IMPORTANT) --------
#         self.joint_order = [
#             "NCCUP1_joint","NTHIGH1_joint","NFOOT1_joint",
#             "NCCUP2_joint","NTHIGH2_joint","NFOOT2_joint",
#             "NCCUP3_joint","NTHIGH3_joint","NFOOT3_joint",
#             "NCCUP4_joint","NTHIGH4_joint","NFOOT4_joint",
#             "NCCUP5_joint","NTHIGH5_joint","NFOOT5_joint",
#             "NCCUP6_joint","NTHIGH6_joint","NFOOT6_joint"
#         ]

#     # -------- SERIAL --------
#     def connect_serial(self):
#         try:
#             self.ser = serial.Serial(self.port, self.baud, timeout=1)
#             self.get_logger().info(f"Connected to {self.port}")
#         except Exception as e:
#             self.get_logger().error(f"Serial failed: {e}")
#             self.ser = None

#     def send_data(self, data):
#         if self.ser is None:
#             self.connect_serial()
#             return

#         try:
#             self.ser.write(data.encode())
#         except Exception as e:
#             self.get_logger().warn(f"Serial error: {e}")
#             try:
#                 self.ser.close()
#             except:
#                 pass
#             self.ser = None

#     # -------- CALLBACK --------
#     def callback(self, msg):

#         joint_map = dict(zip(msg.name, msg.position))

#         angles_deg = []

#         for i, joint in enumerate(self.joint_order):

#             if joint not in joint_map:
#                 self.get_logger().warn(f"{joint} missing!")
#                 return

#             angle_deg = math.degrees(joint_map[joint])

#             # -------- YOUR OFFSET LOGIC --------
#             if i % 3 == 0:      # J1
#                 angle_deg += 45
#             else:               # J2, J3
#                 angle_deg += 90

#             angle_deg = max(0.0, min(180.0, angle_deg))

#             angles_deg.append(int(round(angle_deg)))

#         data = " ".join(map(str, angles_deg)) + "\n"

#         self.send_data(data)

#         self.get_logger().info(f"Sent: {data.strip()}")


# def main(args=None):
#     rclpy.init(args=args)

#     node = JointStateToESP()
#     rclpy.spin(node)

#     node.destroy_node()
#     rclpy.shutdown()


# if __name__ == "__main__":
#     main()