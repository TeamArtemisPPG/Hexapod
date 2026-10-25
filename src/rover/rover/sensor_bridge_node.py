#!/usr/bin/env python3
"""
sensor_bridge_node.py — Aggregate all robot sensor feeds and serve a
structured JSON snapshot via the /get_sensor_snapshot ROS2 service.

Subscriptions:
  /scan                 sensor_msgs/LaserScan     — LiDAR distance sectors
  /odom                 nav_msgs/Odometry          — position + heading
  /camera/camera/image_raw  sensor_msgs/Image      — camera frame (cv object detect)
  /map                  nav_msgs/OccupancyGrid     — map availability flag
  /amcl_pose            geometry_msgs/PoseWithCovarianceStamped — localized pose

Service served:
  /get_sensor_snapshot  rover_interfaces/srv/SensorSnapshot
"""

import json
import math
import os
import threading
from datetime import datetime
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy

import numpy as np
import cv2
from cv_bridge import CvBridge

from sensor_msgs.msg import LaserScan, Image
from nav_msgs.msg import Odometry, OccupancyGrid
from geometry_msgs.msg import PoseWithCovarianceStamped

from rover_interfaces.srv import SensorSnapshot
from rover.frontier_exploration import FrontierExploration

# ── Color detection ranges (HSV) ──────────────────────────────────────────────
COLOR_RANGES = {
    'red': [
        (np.array([0, 120, 70]),   np.array([10, 255, 255])),
        (np.array([170, 120, 70]), np.array([180, 255, 255])),
    ],
    'blue': [
        (np.array([100, 150, 50]), np.array([140, 255, 255])),
    ],
    'green': [
        (np.array([40, 80, 40]),  np.array([80, 255, 255])),
    ],
    'yellow': [
        (np.array([20, 100, 100]), np.array([35, 255, 255])),
    ],
}


def euler_from_quaternion(x, y, z, w) -> float:
    """Return yaw (heading) in degrees from quaternion."""
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.degrees(math.atan2(siny_cosp, cosy_cosp))


class SensorBridgeNode(Node):

    def __init__(self):
        super().__init__('sensor_bridge_node')

        # ── Parameters ────────────────────────────────────────────────────────
        self.declare_parameter('lidar_sector_front_deg', 30.0)
        self.declare_parameter('object_detection_enabled', True)
        self.declare_parameter('snapshot_cache_sec', 0.5)

        front_half = self.get_parameter('lidar_sector_front_deg').value
        self._front_half = math.radians(front_half)
        self._side_deg   = math.radians(front_half)   # ±30° boundary

        # ── State (protected by lock) ─────────────────────────────────────────
        self._lock = threading.Lock()
        self._position     = {'x': 0.0, 'y': 0.0, 'heading_deg': 0.0}
        self._lidar_raw: deque = deque(maxlen=10)      # rolling buffer
        self._lidar_sectors    = {'front': None, 'left': None, 'right': None, 'back': None}
        self._nearest_obstacle = None
        self._camera_summary   = {'objects_detected': [], 'description': 'No frame yet'}
        self._map_ready        = False
        self._nav2_status      = 'unknown'
        self._amcl_pose        = None
        self._frontier_explorer = FrontierExploration(min_cluster_size=4, robot_radius_m=0.35)
        self._frontier_stats   = {'total_frontiers': 0, 'is_fully_mapped': False, 'explored_area_m2': 0.0}
        self._frontier_goals   = []

        self._bridge = CvBridge()

        # ── QoS for map (transient local) ─────────────────────────────────────
        map_qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )

        # ── Subscriptions ─────────────────────────────────────────────────────
        self.create_subscription(LaserScan, '/scan',       self._on_scan,      10)
        self.create_subscription(Odometry,  '/odom',       self._on_odom,      10)
        self.create_subscription(Image,     '/camera/camera/image_raw',
                                 self._on_image, 10)
        self.create_subscription(OccupancyGrid, '/map',   self._on_map,  map_qos)
        self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose',
            self._on_amcl, 10
        )

        # ── Service ───────────────────────────────────────────────────────────
        self.create_service(
            SensorSnapshot, '/get_sensor_snapshot', self._handle_snapshot)

        self.get_logger().info('SensorBridgeNode ready — /get_sensor_snapshot active')

    # ─────────────────────────────────── Callbacks ───────────────────────────

    def _on_scan(self, msg: LaserScan):
        """Compute nearest obstacle and 4-sector summaries from LaserScan."""
        ranges = np.array(msg.ranges, dtype=np.float32)
        # Replace inf/nan with max_range
        max_r = msg.range_max if msg.range_max > 0 else 10.0
        ranges = np.where(np.isfinite(ranges) & (ranges > msg.range_min), ranges, max_r)

        n = len(ranges)
        angle_min = msg.angle_min
        angle_inc = msg.angle_increment

        def sector_min(lo_deg, hi_deg):
            lo = math.radians(lo_deg)
            hi = math.radians(hi_deg)
            indices = []
            for i in range(n):
                a = angle_min + i * angle_inc
                # Normalise to [-π, π]
                a = (a + math.pi) % (2 * math.pi) - math.pi
                if lo <= a <= hi:
                    indices.append(i)
            if not indices:
                return max_r
            return float(np.min(ranges[indices]))

        with self._lock:
            # Rolling buffer of full range arrays
            self._lidar_raw.append(ranges.copy())
            # Average over buffer
            avg = np.mean(np.stack(self._lidar_raw), axis=0)

            # Sectors: front ±30°, left 30°–150°, right −30° to −150°, back ±150°–180°
            front_vals, left_vals, right_vals, back_vals = [], [], [], []
            for i in range(n):
                a = angle_min + i * angle_inc
                a = (a + math.pi) % (2 * math.pi) - math.pi  # [-π,π]
                ad = math.degrees(a)
                v = float(avg[i])
                if -30 <= ad <= 30:
                    front_vals.append(v)
                elif 30 < ad <= 150:
                    left_vals.append(v)
                elif -150 <= ad < -30:
                    right_vals.append(v)
                else:
                    back_vals.append(v)

            def safe_min(lst):
                return round(float(np.min(lst)), 3) if lst else max_r

            self._lidar_sectors = {
                'front': safe_min(front_vals),
                'left':  safe_min(left_vals),
                'right': safe_min(right_vals),
                'back':  safe_min(back_vals),
            }
            self._nearest_obstacle = round(float(np.min(avg)), 3)

    def _on_odom(self, msg: Odometry):
        q = msg.pose.pose.orientation
        heading = euler_from_quaternion(q.x, q.y, q.z, q.w)
        with self._lock:
            self._position = {
                'x':           round(msg.pose.pose.position.x, 3),
                'y':           round(msg.pose.pose.position.y, 3),
                'heading_deg': round(heading, 1),
            }

    def _on_amcl(self, msg: PoseWithCovarianceStamped):
        q = msg.pose.pose.orientation
        heading = euler_from_quaternion(q.x, q.y, q.z, q.w)
        with self._lock:
            self._amcl_pose = {
                'x':           round(msg.pose.pose.position.x, 3),
                'y':           round(msg.pose.pose.position.y, 3),
                'heading_deg': round(heading, 1),
            }

    def _on_image(self, msg: Image):
        """Detect colored objects in camera frame and summarise."""
        if not self.get_parameter('object_detection_enabled').value:
            return
        try:
            frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warn(f'CvBridge error: {e}')
            return

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        h, w = frame.shape[:2]
        cx_img = w // 2
        cy_img = h // 2

        detections = []
        desc_parts = []

        for color, ranges in COLOR_RANGES.items():
            mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
            for lo, hi in ranges:
                mask |= cv2.inRange(hsv, lo, hi)

            contours, _ = cv2.findContours(
                mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if not contours:
                continue

            largest = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(largest)
            if area < 500:
                continue

            x, y, bw, bh = cv2.boundingRect(largest)
            obj_cx = x + bw // 2
            obj_cy = y + bh // 2

            side = 'left' if obj_cx < cx_img else 'right'
            detections.append(f'{color}_object')
            desc_parts.append(
                f'{color.capitalize()} object at pixel ({obj_cx},{obj_cy}), '
                f'area={int(area)}, side={side}'
            )

        with self._lock:
            self._camera_summary = {
                'objects_detected': detections,
                'description': '; '.join(desc_parts) if desc_parts else 'No objects detected',
            }

    def _on_map(self, msg: OccupancyGrid):
        try:
            width = msg.info.width
            height = msg.info.height
            res = msg.info.resolution
            ox = msg.info.origin.position.x
            oy = msg.info.origin.position.y
            data = np.array(msg.data, dtype=np.int8)

            with self._lock:
                pos = self._amcl_pose if self._amcl_pose else self._position
                rx = float(pos.get('x', 0.0))
                ry = float(pos.get('y', 0.0))

            frontiers, _, stats = self._frontier_explorer.extract_frontiers(
                grid_data=data,
                width=width,
                height=height,
                resolution=res,
                origin_x=ox,
                origin_y=oy,
                robot_x=rx,
                robot_y=ry,
            )

            with self._lock:
                self._map_ready = True
                self._frontier_stats = stats
                self._frontier_goals = frontiers
        except Exception as e:
            self.get_logger().warn(f'Error processing map frontiers: {e}')
            with self._lock:
                self._map_ready = True

    # ─────────────────────────────────── Service handler ─────────────────────

    def _handle_snapshot(self, _request, response):
        with self._lock:
            pos = self._amcl_pose if self._amcl_pose else self._position

            # Check if saved map exists on disk
            saved_map_file = '/home/vishal/gz_hex/src/rover/maps/room_map.yaml'
            map_saved = os.path.isfile(saved_map_file)

            # Dynamic obstacle detection (< 0.45m)
            has_dyn_obs = False
            obs_sector = 'none'
            if self._nearest_obstacle is not None and self._nearest_obstacle < 0.45:
                has_dyn_obs = True
                valid_sectors = {k: v for k, v in self._lidar_sectors.items() if v is not None}
                if valid_sectors:
                    obs_sector = min(valid_sectors, key=valid_sectors.get)

            snapshot = {
                'timestamp': datetime.utcnow().isoformat(timespec='seconds'),
                'position': pos,
                'nearest_obstacle_m': self._nearest_obstacle,
                'lidar_sectors': self._lidar_sectors,
                'dynamic_obstacle': {
                    'detected': has_dyn_obs,
                    'distance_m': self._nearest_obstacle,
                    'direction_sector': obs_sector,
                },
                'camera': self._camera_summary,
                'map_ready': self._map_ready,
                'mapping': {
                    'map_ready': self._map_ready,
                    'remaining_frontiers': self._frontier_stats.get('total_frontiers', 0),
                    'is_fully_mapped': self._frontier_stats.get('is_fully_mapped', False),
                    'explored_area_m2': self._frontier_stats.get('explored_area_m2', 0.0),
                    'map_saved': map_saved,
                    'saved_map_file': saved_map_file if map_saved else None,
                },
                'nav2_status': self._nav2_status,
            }
        try:
            response.json_data = json.dumps(snapshot)
            response.success = True
        except Exception as e:
            response.json_data = '{}'
            response.success = False
            self.get_logger().error(f'Snapshot serialisation error: {e}')
        return response

    def set_nav2_status(self, status: str):
        """Called by brain or dispatcher to update nav status in snapshot."""
        with self._lock:
            self._nav2_status = status


def main(args=None):
    rclpy.init(args=args)
    node = SensorBridgeNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
