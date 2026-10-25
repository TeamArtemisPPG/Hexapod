#!/usr/bin/env python3
"""
tool_dispatcher_node.py — Translates LLM tool-call JSON → ROS2 actions/topics.

Service served:
  /execute_tool  rover_interfaces/srv/ExecuteTool

Tools implemented:
  navigate_to_pose          Send Nav2 NavigateToPose goal (x, y, yaw_deg)
  navigate_to_named_location  Resolve predefined waypoint name → navigate
  cancel_navigation         Cancel current Nav2 goal
  get_navigation_status     Return {status, remaining_distance}
  find_object               Activate box_tracker node for N seconds (search mode)
  approach_object           Activate box_tracker in approach mode (continuous)
  stop                      Publish zero cmd_vel immediately
  rotate_in_place           Publish angular cmd_vel for N degrees
  get_sensor_snapshot       Forward call to /get_sensor_snapshot service
  say                       Publish message on /brain/response
"""

import json
import math
import os
import threading
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup

from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, PoseStamped, Quaternion
from std_msgs.msg import String
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus

from rover_interfaces.srv import ExecuteTool, SensorSnapshot
from rover.frontier_exploration import FrontierExploration

# ── Predefined waypoints: {name: [x, y, yaw_deg]} ───────────────────────────
DEFAULT_WAYPOINTS = {
    'home':            [0.0,   0.0,    0.0],
    'origin':          [0.0,   0.0,    0.0],
    'center':          [0.0,   0.0,    0.0],
    'circle_object':   [1.35, -1.35, -45.0],   # Cylinder at (2.0, -2.0)
    'cylinder':        [1.35, -1.35, -45.0],
    'yellow_cylinder': [1.35, -1.35, -45.0],
    'blue_box':        [1.0,   1.0,   45.0],   # Box at (1.5, 1.5)
    'red_box':         [-1.3,  1.3,  135.0],   # Box at (-2.0, 2.0)
    'green_box':       [-1.0, -1.3, -125.0],   # Box at (-1.5, -2.0)
    'inspect_point':   [1.35, -1.35, -45.0],
}


def yaw_to_quaternion(yaw_deg: float) -> Quaternion:
    """Convert yaw angle (degrees) to geometry_msgs/Quaternion."""
    yaw = math.radians(yaw_deg)
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


class ToolDispatcherNode(Node):

    def __init__(self):
        super().__init__('tool_dispatcher_node')

        # ── Parameters ────────────────────────────────────────────────────────
        self.declare_parameter('nav2_timeout_sec', 30.0)
        self.declare_parameter('rotation_speed_rad_s', 0.5)
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')

        self._nav_timeout   = self.get_parameter('nav2_timeout_sec').value
        self._rot_speed     = self.get_parameter('rotation_speed_rad_s').value
        self._cmd_vel_topic = self.get_parameter('cmd_vel_topic').value

        # Merge with default waypoints (could be overridden via parameter server)
        self._waypoints = DEFAULT_WAYPOINTS.copy()

        # ── Nav2 action client ────────────────────────────────────────────────
        self._cb_group = ReentrantCallbackGroup()
        self._nav_client = ActionClient(
            self, NavigateToPose, 'navigate_to_pose',
            callback_group=self._cb_group)

        # Current nav goal handle for cancellation
        self._goal_handle = None
        self._nav_status  = 'idle'
        self._nav_lock    = threading.Lock()

        # ── Publishers ────────────────────────────────────────────────────────
        self._cmd_vel_pub = self.create_publisher(Twist, self._cmd_vel_topic, 10)
        self._response_pub = self.create_publisher(String, '/brain/response', 10)

        # ── Sensor snapshot client ────────────────────────────────────────────
        self._sensor_cli = self.create_client(
            SensorSnapshot, '/get_sensor_snapshot',
            callback_group=self._cb_group)

        # ── Dynamic Obstacle Avoidance Shield (/scan) ─────────────────────────
        self._min_front_dist = 10.0
        self._min_left_dist = 10.0
        self._min_right_dist = 10.0
        self._nearest_scan_dist = 10.0
        self._nearest_scan_angle_deg = 0.0
        self._scan_lock = threading.Lock()
        self._scan_sub = self.create_subscription(
            LaserScan, '/scan', self._on_scan, 10,
            callback_group=self._cb_group)

        # ── Service ───────────────────────────────────────────────────────────
        self.create_service(
            ExecuteTool, '/execute_tool',
            self._handle_execute_tool,
            callback_group=self._cb_group)

        # ── Map & Frontier Exploration ────────────────────────────────────────
        map_qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._map_sub = self.create_subscription(
            OccupancyGrid, '/map', self._on_map, map_qos,
            callback_group=self._cb_group
        )
        self._latest_map = None
        self._map_lock = threading.Lock()
        self._frontier_explorer = FrontierExploration(min_cluster_size=4, robot_radius_m=0.35)

        # ── box_tracker state ─────────────────────────────────────────────────
        self._tracker_active  = False
        self._tracker_timer   = None

        self.get_logger().info('ToolDispatcherNode ready — /execute_tool active with Dynamic Obstacle Shield')

    # ─────────────────────────────────── Service handler ─────────────────────

    def _handle_execute_tool(self, request, response):
        tool   = request.tool_name
        args   = {}
        try:
            if request.args_json.strip():
                args = json.loads(request.args_json)
        except json.JSONDecodeError as e:
            response.success = False
            response.error_message = f'Bad args JSON: {e}'
            return response

        self.get_logger().info(f'[Tool] {tool}({args})')

        dispatch = {
            'navigate_to_pose':           self._tool_navigate_to_pose,
            'navigate_to_named_location': self._tool_navigate_named,
            'cancel_navigation':          self._tool_cancel_navigation,
            'get_navigation_status':      self._tool_nav_status,
            'drive_distance':             self._tool_drive_distance,
            'explore_and_map':            self._tool_explore_and_map,
            'save_map':                   self._tool_save_map,
            'save_location':              self._tool_save_location,
            'get_saved_locations':        self._tool_get_saved_locations,
            'find_object':                self._tool_find_object,
            'approach_object':            self._tool_approach_object,
            'stop':                       self._tool_stop,
            'rotate_in_place':            self._tool_rotate,
            'dance':                      self._tool_dance,
            'get_sensor_snapshot':        self._tool_sensor_snapshot,
            'say':                        self._tool_say,
        }

        handler = dispatch.get(tool)
        if handler is None:
            response.success = False
            response.error_message = f'Unknown tool: {tool}'
            return response

        try:
            result = handler(args)
            response.success     = True
            response.result_json = json.dumps(result)
        except Exception as e:
            response.success       = False
            response.error_message = str(e)
            self.get_logger().error(f'Tool {tool} error: {e}')

        return response

    # ─────────────────────────────────── Tool implementations ────────────────

    def _tool_navigate_to_pose(self, args: dict) -> dict:
        x       = float(args.get('x', 0.0))
        y       = float(args.get('y', 0.0))
        yaw_deg = float(args.get('yaw_deg', 0.0))

        if not self._nav_client.wait_for_server(timeout_sec=5.0):
            raise RuntimeError('Nav2 NavigateToPose action server not available')

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp    = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.orientation = yaw_to_quaternion(yaw_deg)

        # Non-blocking send — brain monitors status via get_navigation_status
        send_future = self._nav_client.send_goal_async(
            goal_msg, feedback_callback=self._nav_feedback)
        send_future.add_done_callback(self._nav_goal_response)

        with self._nav_lock:
            self._nav_status = 'navigating'

        return {'status': 'goal_sent', 'target': {'x': x, 'y': y, 'yaw_deg': yaw_deg}}

    def _nav_feedback(self, feedback_msg):
        # Optionally log remaining distance
        pass

    def _nav_goal_response(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            with self._nav_lock:
                self._nav_status = 'rejected'
            self.get_logger().warn('Nav2 goal rejected')
            return
        with self._nav_lock:
            self._goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._nav_result)

    def _nav_result(self, future):
        result = future.result()
        if result.status == GoalStatus.STATUS_SUCCEEDED:
            with self._nav_lock:
                self._nav_status = 'succeeded'
            self.get_logger().info('Navigation succeeded')
        elif result.status == GoalStatus.STATUS_CANCELED:
            with self._nav_lock:
                self._nav_status = 'cancelled'
        else:
            with self._nav_lock:
                self._nav_status = 'failed'

    def _on_map(self, msg: OccupancyGrid):
        with self._map_lock:
            self._latest_map = msg

    def _navigate_to_pose_blocking(self, x: float, y: float, yaw_deg: float = 0.0, timeout_sec: float = 25.0) -> bool:
        """Send NavigateToPose goal and block until reached or timed out."""
        if not self._nav_client.wait_for_server(timeout_sec=3.0):
            self.get_logger().warn('Nav2 server not available for blocking navigation')
            return False

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.orientation = yaw_to_quaternion(yaw_deg)

        send_future = self._nav_client.send_goal_async(goal_msg)
        t_start = time.time()
        while not send_future.done() and time.time() - t_start < 5.0:
            time.sleep(0.05)

        if not send_future.done() or not send_future.result().accepted:
            self.get_logger().warn(f'Nav2 goal ({x}, {y}) was rejected or timed out')
            return False

        goal_handle = send_future.result()
        with self._nav_lock:
            self._goal_handle = goal_handle
            self._nav_status = 'navigating'

        result_future = goal_handle.get_result_async()
        deadline = time.time() + timeout_sec
        while not result_future.done() and time.time() < deadline:
            time.sleep(0.1)

        if not result_future.done():
            self.get_logger().warn(f'Nav2 goal ({x}, {y}) timed out after {timeout_sec}s. Cancelling...')
            goal_handle.cancel_goal_async()
            with self._nav_lock:
                self._nav_status = 'cancelled'
            return False

        res = result_future.result()
        success = (res.status == GoalStatus.STATUS_SUCCEEDED)
        with self._nav_lock:
            self._nav_status = 'succeeded' if success else 'failed'
        return success

    def _tool_navigate_named(self, args: dict) -> dict:
        name = args.get('name', '').lower()
        wp   = self._waypoints.get(name)
        if wp is None:
            raise ValueError(
                f"Unknown waypoint '{name}'. "
                f"Known: {list(self._waypoints.keys())}"
            )
        return self._tool_navigate_to_pose({'x': wp[0], 'y': wp[1], 'yaw_deg': wp[2]})

    def _tool_cancel_navigation(self, _args: dict) -> dict:
        with self._nav_lock:
            gh = self._goal_handle
        if gh is None:
            return {'cancelled': False, 'reason': 'No active goal'}
        gh.cancel_goal_async()
        with self._nav_lock:
            self._nav_status = 'cancelling'
        return {'cancelled': True}

    def _tool_nav_status(self, _args: dict) -> dict:
        with self._nav_lock:
            status = self._nav_status
        return {'status': status}

    def _tool_stop(self, _args: dict) -> dict:
        """Publish zero Twist to halt robot immediately."""
        twist = Twist()
        self._cmd_vel_pub.publish(twist)
        # Also cancel any active nav goal
        self._tool_cancel_navigation({})
        with self._nav_lock:
            self._nav_status = 'idle'
        return {'stopped': True}

    def _tool_drive_distance(self, args: dict) -> dict:
        """Drive straight by distance_m at speed m/s with dynamic obstacle avoidance shield."""
        distance = float(args.get('distance_m', 1.0))
        speed    = float(args.get('speed', 0.2))

        if abs(speed) < 1e-3:
            speed = 0.2

        if abs(distance) < 0.01:
            return {'driven_distance_m': 0.0, 'status': 'no_motion'}

        duration  = abs(distance) / abs(speed)
        direction = 1.0 if distance > 0 else -1.0

        twist = Twist()
        twist.linear.x = direction * abs(speed)

        rate_hz = 20
        steps   = int(duration * rate_hz)

        actual_steps = 0
        obstacle_halt = False
        avoidance_action = None

        for s in range(steps):
            # Dynamic obstacle avoidance check when driving forward
            if direction > 0:
                with self._scan_lock:
                    front_dist = self._min_front_dist
                    left_dist  = self._min_left_dist
                    right_dist = self._min_right_dist

                if front_dist < 0.40:
                    self._cmd_vel_pub.publish(Twist())
                    obstacle_halt = True
                    self.get_logger().warn(
                        f'⚠️ [Dynamic Obstacle Shield] Front obstacle at {front_dist:.2f}m (< 0.40m)! Halting forward motion.'
                    )
                    # Reactive avoidance: steer toward the side with more clearance
                    turn_deg = 45.0 if left_dist >= right_dist else -45.0
                    self.get_logger().info(f'🔄 [Dynamic Obstacle] Evading obstacle by rotating {turn_deg}°.')
                    self._tool_rotate({'degrees': turn_deg, 'speed': 0.4})
                    avoidance_action = f"Stopped {front_dist:.2f}m from obstacle, steered {turn_deg}° away"
                    break

            self._cmd_vel_pub.publish(twist)
            actual_steps += 1
            time.sleep(1.0 / rate_hz)

        # Stop
        self._cmd_vel_pub.publish(Twist())
        actual_dist = round(direction * actual_steps * (abs(speed) / rate_hz), 2)
        if obstacle_halt:
            return {
                'status': 'obstacle_avoided',
                'driven_distance_m': actual_dist,
                'obstacle_halt': True,
                'action_taken': avoidance_action,
                'message': f'Dynamic obstacle encountered! {avoidance_action}.'
            }
        return {'driven_distance_m': distance, 'status': 'completed'}

    def _tool_rotate(self, args: dict) -> dict:
        """Rotate robot by `degrees` at `speed` rad/s (blocking, timed)."""
        degrees = float(args.get('degrees', 90.0))
        speed   = float(args.get('speed', self._rot_speed))

        # Guard against zero or near-zero speed
        if abs(speed) < 1e-3:
            speed = float(self._rot_speed)

        if abs(degrees) < 0.1:
            return {'rotated_deg': 0.0}

        # Time needed = angle / angular_speed
        duration = abs(math.radians(degrees)) / abs(speed)
        direction = 1.0 if degrees > 0 else -1.0

        twist = Twist()
        twist.angular.z = direction * abs(speed)

        rate_hz = 20
        steps   = int(duration * rate_hz)

        for _ in range(steps):
            self._cmd_vel_pub.publish(twist)
            time.sleep(1.0 / rate_hz)

        # Stop
        self._cmd_vel_pub.publish(Twist())
        return {'rotated_deg': degrees}

    def _tool_dance(self, args: dict) -> dict:
        """
        Perform an entertaining hexapod dance routine (wiggles, steps, spins).
        Rhythmically activates the 18 hexapod leg joints using the tripod gait controller.
        """
        style = args.get('style', 'full')
        self.get_logger().info(f'💃 [Dance] Starting hexapod dance routine (style={style})')

        # Sequence of movements: (linear_x, angular_z, duration_s)
        if style == 'spin':
            routine = [
                (0.0,   0.8, 1.2),
                (0.0,  -0.8, 1.2),
                (0.0,   1.0, 1.5),
                (0.0,  -1.0, 1.5),
            ]
        elif style == 'wiggle':
            routine = [
                (0.0,   0.7, 0.6),
                (0.0,  -0.7, 0.6),
                (0.0,   0.7, 0.6),
                (0.0,  -0.7, 0.6),
                (0.0,   0.7, 0.6),
                (0.0,  -0.7, 0.6),
            ]
        else:  # 'full'
            routine = [
                # 1. Rhythmic side-to-side twist (wiggles)
                (0.0,   0.7, 0.8),
                (0.0,  -0.7, 0.8),
                (0.0,   0.7, 0.6),
                (0.0,  -0.7, 0.6),
                # 2. Step back (moonwalk)
                (-0.25, 0.0, 0.9),
                # 3. Step forward (strut)
                (0.25,  0.0, 0.9),
                # 4. Pirouette spin left and right
                (0.0,   1.0, 1.2),
                (0.0,  -1.0, 1.2),
                # 5. Quick celebratory shimmy
                (0.0,   0.8, 0.5),
                (0.0,  -0.8, 0.5),
            ]

        rate_hz = 20
        total_time = 0.0

        for vx, wz, duration in routine:
            twist = Twist()
            twist.linear.x = float(vx)
            twist.angular.z = float(wz)
            steps = int(duration * rate_hz)
            for _ in range(steps):
                self._cmd_vel_pub.publish(twist)
                time.sleep(1.0 / rate_hz)
            total_time += duration

        # Halt and return to neutral stance
        self._cmd_vel_pub.publish(Twist())
        time.sleep(0.2)

        celebration_msg = "💃 Look at me dance! Hexapod groovy routine completed!"
        self._response_pub.publish(String(data=celebration_msg))
        return {
            'status': 'completed',
            'style': style,
            'duration_s': round(total_time, 1),
            'message': celebration_msg
        }

    def _tool_find_object(self, args: dict) -> dict:
        """
        Rotate 360° while camera looks for target color.
        Returns immediately with status; actual detection is polled via sensor snapshot.
        """
        color   = args.get('color', 'red')
        timeout = float(args.get('max_search_time_s', 10.0))
        # Start a slow rotation search
        twist = Twist()
        twist.angular.z = 0.3

        start = time.time()
        while time.time() - start < timeout:
            self._cmd_vel_pub.publish(twist)
            time.sleep(0.1)

        self._cmd_vel_pub.publish(Twist())
        return {
            'status': 'search_complete',
            'color':  color,
            'elapsed_s': round(time.time() - start, 1),
            'note': 'Check /get_sensor_snapshot camera.objects_detected for results',
        }

    def _tool_approach_object(self, args: dict) -> dict:
        """
        Safely approach an object, landmark, or the nearest obstacle directly using LiDAR & Nav2.
        Supports:
          - 'nearest' or 'obstacle': uses live LiDAR scan to locate the closest obstacle,
            rotates to face it, and drives forward safely until within 0.45m.
          - 'circle' / 'cylinder' / 'yellow': navigates near the cylinder at (2.0, -2.0).
          - 'blue' / 'box_1': navigates near the blue box at (1.5, 1.5).
          - 'red' / 'box_2': navigates near the red box at (-2.0, 2.0).
          - 'green' / 'box_3': navigates near the green box at (-1.5, -2.0).
          - 'center' / 'origin': navigates to origin (0.0, 0.0).
        """
        target = str(args.get('target', args.get('color', 'nearest'))).lower().strip()
        self.get_logger().info(f'🎯 [Approach Object] Target="{target}"')

        # 1. Known landmark mapping
        landmark_map = {
            'circle':          'circle_object',
            'circle object':   'circle_object',
            'circle_object':   'circle_object',
            'cylinder':        'circle_object',
            'yellow':          'circle_object',
            'yellow cylinder': 'circle_object',
            'blue':            'blue_box',
            'blue box':        'blue_box',
            'blue_box':        'blue_box',
            'box_1':           'blue_box',
            'red':             'red_box',
            'red box':         'red_box',
            'red_box':         'red_box',
            'box_2':           'red_box',
            'green':           'green_box',
            'green box':       'green_box',
            'green_box':       'green_box',
            'box_3':           'green_box',
            'center':          'center',
            'centre':          'center',
            'origin':          'origin',
        }

        matched_landmark = landmark_map.get(target)
        if not matched_landmark:
            for k, v in landmark_map.items():
                if k in target:
                    matched_landmark = v
                    break

        if matched_landmark:
            self.get_logger().info(f'🎯 [Approach Object] Navigating to landmark waypoint: {matched_landmark}')
            return self._tool_navigate_named({'name': matched_landmark})

        # 2. Target is 'nearest' (or unspecified obstacle) -> use live LiDAR to locate and approach
        with self._scan_lock:
            dist = getattr(self, '_nearest_scan_dist', 10.0)
            angle_deg = getattr(self, '_nearest_scan_angle_deg', 0.0)

        if dist >= 9.9:
            return {'status': 'failed', 'error': 'No obstacle detected within LiDAR range'}

        if dist <= 0.48:
            msg = f"Already at or near the nearest obstacle ({dist:.2f}m away)."
            self._response_pub.publish(String(data=msg))
            return {
                'status': 'already_near',
                'distance_m': round(dist, 2),
                'message': msg
            }

        self.get_logger().info(
            f'🎯 [Approach Object] Nearest obstacle detected at {dist:.2f}m, bearing {angle_deg:.1f}°. Turning and approaching.'
        )

        # Step 1: Rotate to directly face the nearest obstacle
        if abs(angle_deg) > 5.0:
            self._tool_rotate({'degrees': angle_deg, 'speed': 0.4})
            time.sleep(0.3)

        # Step 2: Drive forward until ~0.45m from the obstacle
        forward_dist = max(0.1, dist - 0.45)
        drive_result = self._tool_drive_distance({'distance_m': forward_dist, 'speed': 0.2})

        with self._scan_lock:
            final_dist = getattr(self, '_min_front_dist', 0.45)

        success_msg = f"Successfully approached the nearest object. Now at {final_dist:.2f}m from it."
        self._response_pub.publish(String(data=success_msg))
        return {
            'status': 'approached_nearest',
            'initial_distance_m': round(dist, 2),
            'final_distance_m': round(final_dist, 2),
            'message': success_msg,
            'drive_details': drive_result
        }

    def _on_scan(self, msg: LaserScan):
        """Ultra-fast scan processing for real-time dynamic obstacle detection & target tracking."""
        ranges = np.array(msg.ranges, dtype=np.float32)
        max_r = float(msg.range_max) if msg.range_max > 0 else 10.0
        ranges = np.where(np.isfinite(ranges) & (ranges > msg.range_min), ranges, max_r)

        n = len(ranges)
        if n == 0:
            return

        angle_min = float(msg.angle_min)
        angle_inc = float(msg.angle_increment)

        front_indices = []
        left_indices = []
        right_indices = []

        front_lim = math.radians(35.0)
        side_hi = math.radians(120.0)

        for i in range(n):
            a = angle_min + i * angle_inc
            a = (a + math.pi) % (2.0 * math.pi) - math.pi
            if -front_lim <= a <= front_lim:
                front_indices.append(i)
            elif front_lim < a <= side_hi:
                left_indices.append(i)
            elif -side_hi <= a < -front_lim:
                right_indices.append(i)

        f_dist = float(np.min(ranges[front_indices])) if front_indices else max_r
        l_dist = float(np.min(ranges[left_indices])) if left_indices else max_r
        r_dist = float(np.min(ranges[right_indices])) if right_indices else max_r

        # Calculate nearest obstacle point across entire 360° scan
        min_idx = int(np.argmin(ranges))
        min_dist = float(ranges[min_idx])
        min_ang = angle_min + min_idx * angle_inc
        min_ang = (min_ang + math.pi) % (2.0 * math.pi) - math.pi
        min_ang_deg = math.degrees(min_ang)

        with self._scan_lock:
            self._min_front_dist = f_dist
            self._min_left_dist  = l_dist
            self._min_right_dist = r_dist
            self._nearest_scan_dist = min_dist
            self._nearest_scan_angle_deg = min_ang_deg

    def _save_map(self, map_name: str = 'room_map') -> dict:
        """
        Saves current OccupancyGrid /map to standard Nav2 PGM + YAML formats.
        Destination: /home/vishal/gz_hex/src/rover/maps/{map_name}.yaml and .pgm
        """
        with self._map_lock:
            current_map = self._latest_map

        if current_map is None:
            self.get_logger().warn('Cannot save map: No map message received yet')
            return {'saved': False, 'error': 'No map received yet'}

        try:
            maps_dir = '/home/vishal/gz_hex/src/rover/maps'
            os.makedirs(maps_dir, exist_ok=True)

            width = current_map.info.width
            height = current_map.info.height
            res = float(current_map.info.resolution)
            ox = float(current_map.info.origin.position.x)
            oy = float(current_map.info.origin.position.y)
            data = np.array(current_map.data, dtype=np.int8).reshape((height, width))

            # ROS OccupancyGrid: -1=unknown, 0=free, 100=occupied
            # Nav2 standard trinary PGM: 254=free, 0=occupied, 205=unknown
            img = np.full((height, width), 205, dtype=np.uint8)
            img[data == 0] = 254
            img[data > 50] = 0

            # Vertical flip (ROS grid origin is bottom-left, PGM is top-left)
            img = np.flipud(img)

            pgm_filename = f'{map_name}.pgm'
            yaml_filename = f'{map_name}.yaml'
            pgm_path = os.path.join(maps_dir, pgm_filename)
            yaml_path = os.path.join(maps_dir, yaml_filename)

            cv2.imwrite(pgm_path, img)

            yaml_content = (
                f"image: {pgm_filename}\n"
                f"resolution: {res}\n"
                f"origin: [{ox:.4f}, {oy:.4f}, 0.0]\n"
                f"negate: 0\n"
                f"occupied_thresh: 0.65\n"
                f"free_thresh: 0.25\n"
                f"mode: trinary\n"
            )
            with open(yaml_path, 'w') as f:
                f.write(yaml_content)

            self.get_logger().info(f'💾 [Map Saved] Complete map saved to {yaml_path}')
            return {
                'saved': True,
                'map_name': map_name,
                'yaml_path': yaml_path,
                'pgm_path': pgm_path,
                'width': width,
                'height': height,
                'resolution': res,
            }
        except Exception as e:
            self.get_logger().error(f'Failed to save map: {e}')
            return {'saved': False, 'error': str(e)}

    def _tool_save_map(self, args: dict) -> dict:
        map_name = args.get('map_name', 'room_map')
        return self._save_map(map_name)

    def _tool_save_location(self, args: dict) -> dict:
        name = args.get('name', '').strip().lower()
        if not name:
            raise ValueError("Location name cannot be empty")

        x = args.get('x')
        y = args.get('y')
        yaw = float(args.get('yaw_deg', 0.0))

        if x is None or y is None:
            # Snapshot current robot position
            snapshot = self._tool_sensor_snapshot({})
            pos = snapshot.get('position', {})
            x = float(pos.get('x', 0.0))
            y = float(pos.get('y', 0.0))
            yaw = float(pos.get('heading_deg', 0.0))

        self._waypoints[name] = [float(x), float(y), float(yaw)]
        self.get_logger().info(f'📍 [Saved Location] "{name}" -> ({x:.2f}, {y:.2f}, {yaw:.1f}°)')
        return {
            'status': 'saved',
            'name': name,
            'coordinates': {'x': round(float(x), 2), 'y': round(float(y), 2), 'yaw_deg': round(yaw, 1)},
            'known_locations': list(self._waypoints.keys()),
        }

    def _tool_get_saved_locations(self, _args: dict) -> dict:
        return {'locations': self._waypoints}

    def _tool_explore_and_map(self, args: dict) -> dict:
        """
        True Frontier Exploration:
        Analyzes live SLAM OccupancyGrid (/map) to locate boundaries between
        explored free space and unknown territory.
        Navigates to the highest-scoring frontier and performs a 360° LiDAR sweep
        to uncover unknown regions. Automatically saves map upon completion.
        """
        mode = args.get('mode', 'frontier')  # 'frontier' (single step) or 'full' (complete mapping)
        raw_max = args.get('max_frontiers', 6 if mode == 'full' else 1)
        try:
            max_frontiers_to_explore = int(raw_max)
        except (ValueError, TypeError):
            max_frontiers_to_explore = 6 if mode == 'full' else 1

        # If LLM passes 0 or negative, interpret as continuous mapping (up to 15 frontiers)
        if max_frontiers_to_explore <= 0:
            max_frontiers_to_explore = 15 if mode == 'full' else 1

        self.get_logger().info(
            f'🗺️ [Frontier Exploration] Starting mode={mode}, max_frontiers={max_frontiers_to_explore}'
        )

        visited_goals = []
        cycles = 0
        last_target = None
        stats = {'total_frontiers': 0, 'is_fully_mapped': False, 'explored_area_m2': 0.0}

        # CRITICAL: Always define robot_x and robot_y BEFORE the while loop to prevent UnboundLocalError
        robot_x, robot_y = 0.0, 0.0
        try:
            snapshot = self._tool_sensor_snapshot({})
            pos = snapshot.get('position', {})
            robot_x = float(pos.get('x', 0.0))
            robot_y = float(pos.get('y', 0.0))
        except Exception as e:
            self.get_logger().warn(f'Could not fetch initial robot pose: {e}')

        while cycles < max_frontiers_to_explore:
            cycles += 1

            # Fetch fresh robot pose
            try:
                snapshot = self._tool_sensor_snapshot({})
                pos = snapshot.get('position', {})
                robot_x = float(pos.get('x', 0.0))
                robot_y = float(pos.get('y', 0.0))
            except Exception as e:
                self.get_logger().warn(f'Could not fetch robot pose: {e}')

            # Extract frontiers from latest map
            with self._map_lock:
                current_map = self._latest_map

            if current_map is None:
                self.get_logger().info('🗺️ No map received yet. Doing initial 360° sweep...')
                self._tool_rotate({'degrees': 360.0, 'speed': 0.5})
                self._tool_drive_distance({'distance_m': 0.8, 'speed': 0.2})
                time.sleep(1.0)
                continue

            width = current_map.info.width
            height = current_map.info.height
            res = current_map.info.resolution
            ox = current_map.info.origin.position.x
            oy = current_map.info.origin.position.y
            data = np.array(current_map.data, dtype=np.int8)

            frontiers, _, stats = self._frontier_explorer.extract_frontiers(
                grid_data=data,
                width=width,
                height=height,
                resolution=res,
                origin_x=ox,
                origin_y=oy,
                robot_x=robot_x,
                robot_y=robot_y,
            )

            # Filter out frontiers already visited in this session
            available_frontiers = []
            for f in frontiers:
                already_visited = False
                for vg in visited_goals:
                    if math.hypot(f['x'] - vg[0], f['y'] - vg[1]) < 0.6:
                        already_visited = True
                        break
                if not already_visited:
                    available_frontiers.append(f)

            if len(available_frontiers) == 0:
                self.get_logger().info('🎉 [Frontier Exploration] No reachable frontiers remaining. Area is fully mapped!')
                save_res = self._save_map('room_map')
                return {
                    'status': 'mapping_completed',
                    'is_fully_mapped': True,
                    'map_saved': save_res.get('saved', False),
                    'map_path': save_res.get('yaml_path', ''),
                    'remaining_frontiers': 0,
                    'explored_area_m2': stats.get('explored_area_m2', 0.0),
                    'message': (
                        'All reachable frontiers have been mapped and saved to disk! '
                        'The environment is completely mapped and ready for subsequent navigation tasks.'
                    ),
                    'cycles_completed': cycles - 1,
                }

            # Select best frontier
            target = available_frontiers[0]
            tx, ty = target['x'], target['y']
            tyaw = target['yaw_deg']
            visited_goals.append((tx, ty))
            last_target = target

            self.get_logger().info(
                f'🗺️ [Frontier Exploration] Frontier {cycles}/{max_frontiers_to_explore}: '
                f'Target ({tx}, {ty}) | Dist: {target["distance"]}m | Area: {target["area"]} cells'
            )

            # Navigate to frontier using Nav2
            nav_success = self._navigate_to_pose_blocking(tx, ty, tyaw, timeout_sec=25.0)

            if not nav_success:
                self.get_logger().warn(
                    f'Nav2 goal to ({tx}, {ty}) timed out or was rejected. Using direct approach...'
                )
                # Fallback: Turn towards frontier and drive forward into it
                turn_needed = tyaw
                self._tool_rotate({'degrees': turn_needed, 'speed': 0.5})
                step = min(target['distance'], 0.8)
                self._tool_drive_distance({'distance_m': step, 'speed': 0.2})

            # Perform 360° LiDAR sweep at the frontier to capture surrounding unknown space
            self.get_logger().info('🗺️ [Frontier Exploration] Sweeping 360° with LiDAR into unknown region...')
            self._tool_rotate({'degrees': 360.0, 'speed': 0.5})

            # Wait briefly for SLAM Toolbox to incorporate the new scans
            time.sleep(1.5)

            if mode == 'frontier':
                break

        # Re-assess frontiers after the exploration step(s)
        with self._map_lock:
            final_map = self._latest_map

        final_stats = stats
        rem_count = 0
        if final_map is not None:
            data = np.array(final_map.data, dtype=np.int8)
            f_list, _, s = self._frontier_explorer.extract_frontiers(
                grid_data=data,
                width=final_map.info.width,
                height=final_map.info.height,
                resolution=final_map.info.resolution,
                origin_x=final_map.info.origin.position.x,
                origin_y=final_map.info.origin.position.y,
                robot_x=robot_x,
                robot_y=robot_y,
            )
            rem_count = len(f_list)
            final_stats = s

        is_done = (rem_count == 0)

        # Auto-save map on completion or after cycles
        save_res = self._save_map('room_map')

        target_desc = f"({last_target['x']}, {last_target['y']})" if last_target else "initial position"
        return {
            'status': 'mapping_completed' if is_done else 'frontier_explored',
            'is_fully_mapped': is_done,
            'map_saved': save_res.get('saved', False),
            'map_path': save_res.get('yaml_path', ''),
            'remaining_frontiers': rem_count,
            'explored_area_m2': final_stats.get('explored_area_m2', 0.0),
            'last_target': last_target,
            'summary': (
                f'Explored frontier at {target_desc}. '
                f'Completed 360° LiDAR mapping. Remaining frontiers: {rem_count}. '
                f'Explored area: {final_stats.get("explored_area_m2", 0.0)} m². '
                + (f"Map saved to {save_res.get('yaml_path')}." if save_res.get('saved') else "")
            )
        }

    def _tool_sensor_snapshot(self, _args: dict) -> dict:
        """Proxy call to /get_sensor_snapshot service."""
        if not self._sensor_cli.wait_for_service(timeout_sec=3.0):
            raise RuntimeError('/get_sensor_snapshot service not available')
        req    = SensorSnapshot.Request()
        future = self._sensor_cli.call_async(req)
        # Wait for future with deadline (do not call spin_until_future_complete on spinning executor)
        deadline = time.time() + 5.0
        while not future.done() and time.time() < deadline:
            time.sleep(0.05)
        if not future.done() or future.result() is None:
            raise RuntimeError('Sensor snapshot call timed out')
        return json.loads(future.result().json_data)

    def _tool_say(self, args: dict) -> dict:
        message = args.get('message', '')
        self._response_pub.publish(String(data=message))
        self.get_logger().info(f'[Brain says]: {message}')
        return {'published': True, 'message': message}


def main(args=None):
    rclpy.init(args=args)
    node = ToolDispatcherNode()
    from rclpy.executors import MultiThreadedExecutor
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
