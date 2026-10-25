#!/usr/bin/env python3
"""
nvidia_brain_node.py — Autonomous LLM brain for the hexapod robot.

Listens to /brain/query (std_msgs/String) for natural-language task strings.
Fetches sensor state, calls NVIDIA NIM (OpenAI-compatible), interprets tool
calls in an agentic loop, dispatches them via /execute_tool, and publishes
the final response to /brain/response.

NVIDIA NIM free-tier model: meta/llama-3.1-70b-instruct
API: https://integrate.api.nvidia.com/v1  (OpenAI-compatible)
Env: export NVIDIA_API_KEY="nvapi-xxxxxxxxxxxx"
"""

import json
import os
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup

from std_msgs.msg import String

from rover_interfaces.srv import SensorSnapshot, ExecuteTool

# ── Try importing openai (NVIDIA NIM uses the same client) ───────────────────
try:
    from openai import OpenAI
except ImportError:
    raise ImportError(
        "openai package not found. Install with: pip install openai"
    )

# ─────────────────────────────────── System Prompt ───────────────────────────

SYSTEM_PROMPT = """You are Zoe, an intelligent, highly autonomous 6-legged hexapod robot operating \
in a Gazebo simulation environment. You are equipped with a LiDAR scanner, forward camera, \
SLAM Toolbox mapping, Nav2 autonomous navigation, and hexapod inverse kinematics tripod gait locomotion.

YOUR OBJECTIVE:
Given natural-language user commands (e.g. "round the entire area", "explore and save map", \
"patrol the room", "find the red box and return to start"), you think, plan, and execute \
autonomously using your tools without needing separate nodes or human assistance.

CORE AUTONOMY PRINCIPLES:
1. ALWAYS inspect [CURRENT SENSOR STATE] before deciding actions:
   - Check if the environment is already mapped (`is_fully_mapped` or `map_saved`).
   - Check if dynamic obstacles are present (`dynamic_obstacle.detected`).
   - Note your current (x, y) coordinates and heading.

2. MAPPING & ROUNDING THE ENTIRE AREA:
   - When asked to "round the entire area", "explore", "map the room", or "scan the environment":
     * If the map is not yet complete (`is_fully_mapped` is False or `remaining_frontiers` > 0):
       Call 'explore_and_map' with mode='full', max_frontiers=6. (Never pass max_frontiers=0).
     * The 'explore_and_map' tool navigates to unexplored frontier boundaries, executes 360° LiDAR sweeps, \
       unveils the occupancy grid, and AUTOMATICALLY saves the map to disk (/home/vishal/gz_hex/src/rover/maps/room_map.yaml).
     * If frontiers still remain after one round, call 'explore_and_map' again until fully mapped.
     * Once mapped (`is_fully_mapped` is True or remaining_frontiers == 0), the map is saved.
     * If the user specifically said "round the entire area", after mapping you can also perform a patrol lap across the mapped area \
       using 'navigate_to_pose' to complete the round!

3. MAP-GROUNDED REASONING FOR NEXT TASKS:
   - Once mapped, NEVER restart mapping! All subsequent tasks (navigation, inspection, object searching, patrol) \
     MUST utilize the saved map geometry.
   - Use 'navigate_to_pose' or 'navigate_to_named_location' with coordinates in the map frame.
   - You can save important spots using 'save_location' (e.g. save_location(name="red_box_location")) \
     so they can be revisited easily.

4. OBJECTS, LANDMARKS & APPROACHING OBJECTS:
   The simulated environment contains the following landmark objects:
   - "circle object" / "cylinder" / "yellow object": Located at (x: 2.0, y: -2.0). Approach waypoint: (x: 1.35, y: -1.35, yaw: -45°).
   - "blue box": Located at (x: 1.5, y: 1.5). Approach waypoint: (x: 1.0, y: 1.0, yaw: 45°).
   - "red box": Located at (x: -2.0, y: 2.0). Approach waypoint: (x: -1.3, y: 1.3, yaw: 135°).
   - "green box": Located at (x: -1.5, y: -2.0). Approach waypoint: (x: -1.0, y: -1.3, yaw: -125°).
   - "centre of the map" / "center" / "origin": Located at (x: 0.0, y: 0.0, yaw: 0°).

   When asked to approach or go near any object:
   - For "circle object" or "cylinder": Call 'approach_object' with target='circle' (or use navigate_to_named_location(name='circle_object')).
   - For "nearest object" or "nearest obstacle": Call 'approach_object' with target='nearest'. It directly reads the 360° LiDAR scan, turns to face the nearest obstacle, and safely moves to 0.45m from it!
   - For other objects (blue box, red box, green box): Call 'approach_object' with target='blue_box', target='red_box', etc.

5. DYNAMIC OBSTACLE AVOIDANCE & ADAPTATION:
   - Dynamic obstacle avoidance is actively enforced by both Nav2 and the dynamic obstacle shield.
   - If an obstacle blocks your forward path (nearest_obstacle_m < 0.40m), halt, check left/right clearance, \
     and steer around the obstacle, or plan an alternate route.
   - If a tool reports 'obstacle_avoided' or navigation fails, DO NOT repeat the exact same blocked action. \
     Adapt: rotate to open space or pick an alternate waypoint.

6. DANCING & CELEBRATION:
   - When asked to dance, celebrate, wiggle, or groove (e.g. "hey zoe dance", "dance for me", "do a dance"):
     Call 'dance' with style='full' (or 'wiggle' or 'spin'). When finished, celebrate with the user!

7. EXECUTION & COMMUNICATION:
   - Call only ONE tool per turn.
   - Do NOT pass max_frontiers=0. Use max_frontiers=6 for full exploration.
   - If a tool fails with an error, do NOT retry the exact same failing tool call in an endless loop. \
     Analyze what went wrong, adapt, or report to the user.
   - When your task is complete, provide a clear, concise summary of what you did using the 'say' tool or final text.

8. TASK COMPLETION & STOPPING:
   - CRITICAL: Once the user's task is completed (e.g. mapping is finished and map is saved, or navigation goal reached, or object approached, or dance finished, or area rounded), YOU MUST STOP CALLING TOOLS IMMEDIATELY!
   - Output your final textual summary explaining that the task is finished.
   - Do NOT poll get_navigation_status or call explore_and_map again once the task has succeeded.
"""

# ─────────────────────────────────── Tool Schema ─────────────────────────────

TOOLS = [
    {
        "type": "function",
        "function": {
            "name": "get_sensor_snapshot",
            "description": (
                "Get the current state of all robot sensors: LiDAR sector distances, "
                "camera object detections, current position (x,y,heading), map readiness, "
                "and navigation status. Call this FIRST before any action."
            ),
            "parameters": {"type": "object", "properties": {}},
        },
    },
    {
        "type": "function",
        "function": {
            "name": "drive_distance",
            "description": (
                "Drive straight forward or backward. MUST use this tool for commands like "
                "'go forward', 'move forward', 'move straight', 'move backward', 'back up'. "
                "Default distance is 1.0 meter if not specified by user. "
                "Positive distance = drive forward, Negative distance = drive backward."
            ),
            "parameters": {
                "type": "object",
                "properties": {
                    "distance_m": {
                        "type": "number",
                        "description": "Distance in meters to drive (default: 1.0 for forward, -1.0 for backward)",
                        "default": 1.0,
                    },
                    "speed": {
                        "type": "number",
                        "description": "Linear speed in m/s (default: 0.2)",
                        "default": 0.2,
                    },
                },
            },
        },
    },
    {
        "type": "function",
        "function": {
            "name": "explore_and_map",
            "description": (
                "Autonomously explore and map unknown/unmapped areas using Frontier Exploration. "
                "Detects boundaries between explored free space and unknown space on the SLAM occupancy grid, "
                "navigates to unexplored boundaries with Nav2, and performs 360° LiDAR sweeps to reveal unknown space. "
                "AUTOMATICALLY saves the map to disk (/home/vishal/gz_hex/src/rover/maps/room_map.yaml) upon completion. "
                "Returns remaining_frontiers count, explored area in m², and whether the area is fully mapped. "
                "Use mode='full' (default max_frontiers=6) to systematically map the entire place. Do NOT pass max_frontiers=0."
            ),
            "parameters": {
                "type": "object",
                "properties": {
                    "mode": {
                        "type": "string",
                        "enum": ["full", "frontier"],
                        "description": "'full' systematically maps until all frontiers are explored; 'frontier' maps 1 frontier.",
                        "default": "full",
                    },
                    "max_frontiers": {
                        "type": "integer",
                        "description": "Number of frontiers to explore in this step (default: 6 for full, 1 for frontier). Valid range: 1 to 15. Do NOT pass 0.",
                        "default": 6,
                    },
                },
            },
        },
    },
    {
        "type": "function",
        "function": {
            "name": "save_map",
            "description": "Explicitly save the current SLAM occupancy grid map to disk (.yaml and .pgm formats in /home/vishal/gz_hex/src/rover/maps/).",
            "parameters": {
                "type": "object",
                "properties": {
                    "map_name": {
                        "type": "string",
                        "description": "Base filename for the saved map (default: room_map)",
                        "default": "room_map",
                    },
                },
            },
        },
    },
    {
        "type": "function",
        "function": {
            "name": "save_location",
            "description": "Save a named landmark or waypoint (e.g. 'home', 'red_box_location', 'corner_1') to memory for subsequent navigation.",
            "parameters": {
                "type": "object",
                "properties": {
                    "name": {"type": "string", "description": "Name for the location (e.g. 'red_box_location', 'patrol_point_1')"},
                    "x": {"type": "number", "description": "X coordinate in map frame (omit to use current position)"},
                    "y": {"type": "number", "description": "Y coordinate in map frame (omit to use current position)"},
                    "yaw_deg": {"type": "number", "description": "Heading in degrees (omit to use current heading)"},
                },
                "required": ["name"],
            },
        },
    },
    {
        "type": "function",
        "function": {
            "name": "get_saved_locations",
            "description": "Retrieve all known and saved named locations/waypoints.",
            "parameters": {"type": "object", "properties": {}},
        },
    },
    {
        "type": "function",
        "function": {
            "name": "navigate_to_pose",
            "description": (
                "Navigate the robot to a specific (x,y) map coordinate using Nav2. "
                "Returns immediately after sending the goal; use get_navigation_status to track."
            ),
            "parameters": {
                "type": "object",
                "properties": {
                    "x":       {"type": "number", "description": "X coordinate in map frame (metres)"},
                    "y":       {"type": "number", "description": "Y coordinate in map frame (metres)"},
                    "yaw_deg": {"type": "number", "description": "Desired heading in degrees (0=East, 90=North)"},
                },
                "required": ["x", "y"],
            },
        },
    },
    {
        "type": "function",
        "function": {
            "name": "navigate_to_named_location",
            "description": "Navigate to a predefined named waypoint (e.g. 'home', 'inspect_point', 'origin').",
            "parameters": {
                "type": "object",
                "properties": {
                    "name": {"type": "string", "description": "Waypoint name"},
                },
                "required": ["name"],
            },
        },
    },
    {
        "type": "function",
        "function": {
            "name": "cancel_navigation",
            "description": "Cancel the current Nav2 navigation goal.",
            "parameters": {"type": "object", "properties": {}},
        },
    },
    {
        "type": "function",
        "function": {
            "name": "get_navigation_status",
            "description": "Get the current navigation status (idle, navigating, succeeded, failed, cancelled).",
            "parameters": {"type": "object", "properties": {}},
        },
    },
    {
        "type": "function",
        "function": {
            "name": "find_object",
            "description": (
                "Slowly rotate the robot while searching for an object of the given color. "
                "Runs for max_search_time_s seconds then stops."
            ),
            "parameters": {
                "type": "object",
                "properties": {
                    "color": {
                        "type": "string",
                        "enum": ["red", "blue", "green", "yellow"],
                        "description": "Color of the target object",
                    },
                    "max_search_time_s": {
                        "type": "number",
                        "default": 10.0,
                        "description": "How long to search in seconds",
                    },
                },
                "required": ["color"],
            },
        },
    },
    {
        "type": "function",
        "function": {
            "name": "approach_object",
            "description": (
                "Safely approach an object, landmark, or the nearest obstacle directly using LiDAR & Nav2. "
                "For target='nearest', uses live LiDAR to find the closest obstacle, rotates to face it, and drives until 0.45m away. "
                "For target='circle' (or 'cylinder'), navigates near the cylinder at (2.0, -2.0). "
                "Also supports 'blue_box', 'red_box', 'green_box', or 'center'."
            ),
            "parameters": {
                "type": "object",
                "properties": {
                    "target": {
                        "type": "string",
                        "description": "Object to approach: 'nearest', 'circle', 'cylinder', 'blue_box', 'red_box', 'green_box', or 'center'.",
                        "default": "nearest",
                    },
                    "color": {
                        "type": "string",
                        "description": "Optional color alias (e.g. 'yellow' for circle, 'red', 'blue', 'green')",
                    },
                },
            },
        },
    },
    {
        "type": "function",
        "function": {
            "name": "stop",
            "description": "Immediately stop all robot motion and cancel navigation.",
            "parameters": {"type": "object", "properties": {}},
        },
    },
    {
        "type": "function",
        "function": {
            "name": "rotate_in_place",
            "description": "Rotate the robot in place by a specified number of degrees.",
            "parameters": {
                "type": "object",
                "properties": {
                    "degrees": {
                        "type": "number",
                        "description": "Degrees to rotate. Positive=counter-clockwise, Negative=clockwise.",
                    },
                    "speed": {
                        "type": "number",
                        "description": "Rotation speed in rad/s (default 0.5)",
                        "default": 0.5,
                    },
                },
                "required": ["degrees"],
            },
        },
    },
    {
        "type": "function",
        "function": {
            "name": "dance",
            "description": (
                "Perform an entertaining hexapod dance routine (wiggles, steps, spins) "
                "using the 18-joint tripod gait controller. Call this whenever the user asks to dance, groove, or celebrate."
            ),
            "parameters": {
                "type": "object",
                "properties": {
                    "style": {
                        "type": "string",
                        "enum": ["full", "wiggle", "spin"],
                        "description": "Choreography style: 'full' (all moves), 'wiggle' (rhythmic twists), or 'spin' (pirouettes)",
                        "default": "full",
                    },
                },
            },
        },
    },
    {
        "type": "function",
        "function": {
            "name": "say",
            "description": "Send a text message to the user (published on /brain/response).",
            "parameters": {
                "type": "object",
                "properties": {
                    "message": {"type": "string", "description": "Message to display to user"},
                },
                "required": ["message"],
            },
        },
    },
]


# ─────────────────────────────────── Brain Node ───────────────────────────────

class NvidiaBrainNode(Node):

    def __init__(self):
        super().__init__('nvidia_brain_node')

        # ── Parameters ────────────────────────────────────────────────────────
        self.declare_parameter('nvidia_api_key_env', 'NVIDIA_API_KEY')
        self.declare_parameter('model', 'meta/llama-3.2-11b-vision-instruct')
        self.declare_parameter('temperature', 0.3)
        self.declare_parameter('max_tokens', 1024)
        self.declare_parameter('max_agentic_iterations', 25)
        self.declare_parameter('history_length', 10)

        model        = self.get_parameter('model').value
        temperature  = self.get_parameter('temperature').value
        max_tokens   = self.get_parameter('max_tokens').value
        self._max_iter = self.get_parameter('max_agentic_iterations').value
        self._max_hist = self.get_parameter('history_length').value

        # ── NVIDIA NIM client ─────────────────────────────────────────────────
        api_key_env = self.get_parameter('nvidia_api_key_env').value
        api_key     = os.environ.get(api_key_env)
        if not api_key:
            # Check workspace and standard paths for .brain_secrets
            candidates = [
                os.path.join(os.getcwd(), '.brain_secrets'),
                os.path.expanduser('/home/vishal/gz_hex/.brain_secrets'),
            ]
            for p in candidates:
                if os.path.isfile(p):
                    try:
                        with open(p, 'r') as sf:
                            for line in sf:
                                line = line.strip()
                                if line.startswith(f'{api_key_env}='):
                                    api_key = line.split('=', 1)[1].strip().strip('"\'')
                                    break
                        if api_key:
                            self.get_logger().info(f'Loaded {api_key_env} from {p}')
                            break
                    except Exception:
                        pass

        if not api_key:
            self.get_logger().fatal(
                f"NVIDIA_API_KEY not set! "
                f"Export: export {api_key_env}=nvapi-xxxx"
            )
            raise RuntimeError(f"Missing env var: {api_key_env}")

        self._llm = OpenAI(
            base_url='https://integrate.api.nvidia.com/v1',
            api_key=api_key,
        )
        self._model       = model
        self._temperature = temperature
        self._max_tokens  = max_tokens

        self.get_logger().info(f'Using NVIDIA NIM model: {self._model}')

        # ── Conversation memory ───────────────────────────────────────────────
        self._history: list = []
        self._hist_lock     = threading.Lock()

        # ── ROS2 interfaces ───────────────────────────────────────────────────
        self._cb_group = ReentrantCallbackGroup()

        self._query_sub = self.create_subscription(
            String, '/brain/query', self._on_query, 10,
            callback_group=self._cb_group)

        self._response_pub = self.create_publisher(String, '/brain/response', 10)

        self._sensor_cli = self.create_client(
            SensorSnapshot, '/get_sensor_snapshot',
            callback_group=self._cb_group)

        self._tool_cli = self.create_client(
            ExecuteTool, '/execute_tool',
            callback_group=self._cb_group)

        # ── Busy guard (prevent overlapping queries) ───────────────────────────
        self._busy      = False
        self._busy_lock = threading.Lock()

        self.get_logger().info(
            'NvidiaBrainNode ready. Send queries to /brain/query'
        )

    # ─────────────────────────────────── Query handler ───────────────────────

    def _on_query(self, msg: String):
        raw_query = (msg.data or '').strip()
        cleaned_query = raw_query.strip(' "\'.,!?:;')
        if not cleaned_query:
            return

        # ── Wake Word Filter: Bot's name is Zoe (supports voice & text) ───────
        import re
        wake_match = re.match(
            r'^(?:hey\s+|hi\s+|ok\s+)?(?:zoe|zoey|zoi|joey|chloe|joy)[,\s:]*\s*(.*)$',
            cleaned_query,
            re.IGNORECASE
        )
        if not wake_match:
            self.get_logger().info(
                f"[Ignored - Missing wake word] Received: '{raw_query}'. Expected 'Hey Zoe ...'"
            )
            self._publish("👂 Hello! I'm Zoe. Please say 'Hey Zoe' before your command.")
            return

        user_query = wake_match.group(1).strip()
        if not user_query:
            self._publish("👋 Hey! I'm Zoe, your hexapod robot. What would you like me to do?")
            return

        with self._busy_lock:
            if self._busy:
                self._publish(
                    "⚠️ Still processing previous task. Please wait.")
                return
            self._busy = True

        self.get_logger().info(f'[Zoe Query] {user_query}')
        # Run reasoning loop in executor thread so we don't block spin
        threading.Thread(
            target=self._reasoning_loop,
            args=(user_query,),
            daemon=True,
        ).start()

    def _reasoning_loop(self, user_query: str):
        try:
            self._run_agentic_loop(user_query)
        except Exception as e:
            self.get_logger().error(f'Brain error: {e}')
            self._publish(f'❌ Brain error: {e}')
        finally:
            with self._busy_lock:
                self._busy = False

    def _run_agentic_loop(self, user_query: str):
        # 1. Fetch sensor snapshot (best-effort)
        sensor_ctx = self._get_sensor_snapshot_sync()

        # 2. Build message list
        with self._hist_lock:
            history_slice = list(self._history[-self._max_hist:])

        messages = [
            {'role': 'system', 'content': SYSTEM_PROMPT},
            {
                'role': 'system',
                'content': (
                    f'[CURRENT SENSOR STATE]\n{sensor_ctx}\n'
                    f'Use this to ground your reasoning.'
                )
            },
            *history_slice,
            {'role': 'user', 'content': user_query},
        ]

        final_response = None
        last_failed_call = None
        repeat_error_count = 0

        # 3. Agentic loop
        for iteration in range(self._max_iter):
            self.get_logger().info(
                f'[Brain] LLM call iteration {iteration + 1}/{self._max_iter}'
            )

            try:
                response = self._llm.chat.completions.create(
                    model=self._model,
                    messages=messages,
                    tools=TOOLS,
                    tool_choice='auto',
                    max_tokens=self._max_tokens,
                    temperature=self._temperature,
                )
            except Exception as e:
                self.get_logger().error(f'NVIDIA API error: {e}')
                self._publish(f'❌ NVIDIA API error: {e}')
                return

            choice = response.choices[0]

            # ── Tool calls ────────────────────────────────────────────────────
            if choice.message.tool_calls:
                # Take first tool call (NVIDIA NIM requires 1 tool call per turn)
                tc = choice.message.tool_calls[0]
                fn_name = tc.function.name
                fn_args = tc.function.arguments
                self.get_logger().info(f'[Tool call] {fn_name}({fn_args})')

                result_json = self._dispatch_tool_sync(fn_name, fn_args)
                self.get_logger().info(f'[Tool result] {fn_name} → {result_json}')

                # Reconstruct assistant message with exactly 1 tool call
                tc_dict = {
                    'id': tc.id,
                    'type': 'function',
                    'function': {
                        'name': fn_name,
                        'arguments': fn_args,
                    }
                }
                messages.append({
                    'role': 'assistant',
                    'tool_calls': [tc_dict],
                })
                messages.append({
                    'role': 'tool',
                    'tool_call_id': tc.id,
                    'content': str(result_json),
                })

                # Prevent infinite retry loop if tool keeps failing
                if '"error":' in str(result_json):
                    call_sig = (fn_name, fn_args)
                    if call_sig == last_failed_call:
                        repeat_error_count += 1
                    else:
                        last_failed_call = call_sig
                        repeat_error_count = 1

                    if repeat_error_count >= 2:
                        self.get_logger().warn(
                            f'Tool {fn_name} failed repeatedly ({repeat_error_count} times). Adding advisory.'
                        )
                        messages.append({
                            'role': 'system',
                            'content': (
                                f"System Advisory: The tool '{fn_name}' failed with error: {result_json}. "
                                f"Do NOT call this exact tool with identical arguments again. "
                                f"Adapt your plan, try another tool, or formulate your final summary."
                            )
                        })

                # If the agent called say, it is speaking its final message to the user: finish immediately!
                if fn_name == 'say':
                    try:
                        args_d = json.loads(fn_args)
                        final_response = args_d.get('message', '')
                    except Exception:
                        final_response = 'Task completed.'
                    self.get_logger().info(f'[Brain] Task finished via say tool: {final_response}')
                    break

                # If mapping completed, force next turn to be final summary
                if 'mapping_completed' in str(result_json) or '"is_fully_mapped": true' in str(result_json):
                    self.get_logger().info('🎉 [Brain] Mapping complete detected! Enforcing finalization.')
                    messages.append({
                        'role': 'system',
                        'content': (
                            'COMPLETION NOTICE: Frontier exploration is 100% complete and the map has been saved! '
                            'The entire area is mapped. DO NOT call any more tools. '
                            'Provide your final summary to the user now.'
                        )
                    })

                # If navigation completed, instruct to wrap up
                if fn_name == 'get_navigation_status' and '"status": "succeeded"' in str(result_json):
                    messages.append({
                        'role': 'system',
                        'content': (
                            'Navigation goal reached successfully! '
                            'DO NOT call any more tools. Provide your final summary to the user now.'
                        )
                    })

                # If dance completed, instruct to wrap up
                if fn_name == 'dance':
                    messages.append({
                        'role': 'system',
                        'content': (
                            'Dance routine successfully finished! '
                            'DO NOT call any more tools. Conclude with a cheerful celebratory message to the user now.'
                        )
                    })

                # If approach_object completed approaching directly, wrap up
                if fn_name == 'approach_object' and ('"approached_nearest"' in str(result_json) or '"already_near"' in str(result_json)):
                    messages.append({
                        'role': 'system',
                        'content': (
                            'Object approached successfully! '
                            'DO NOT call any more tools. Provide your final summary to the user now.'
                        )
                    })

                # Continue loop to let model process tool results
                continue

            # ── Final text response ───────────────────────────────────────────
            final_response = (choice.message.content or '').strip()
            if final_response:
                self._publish(final_response)
            break

        else:
            # Exhausted iterations without text response
            final_response = (
                '⚠️ Task requires more steps than allowed. '
                'Partial actions may have been taken.'
            )
            self._publish(final_response)

        # 4. Update conversation history
        with self._hist_lock:
            self._history.append({'role': 'user',      'content': user_query})
            self._history.append({'role': 'assistant',
                                  'content': final_response or 'Task completed.'})
            # Trim to max length
            if len(self._history) > self._max_hist * 2:
                self._history = self._history[-(self._max_hist * 2):]

    # ─────────────────────────────────── Sync helpers ────────────────────────

    def _get_sensor_snapshot_sync(self) -> str:
        """Call /get_sensor_snapshot synchronously (blocking)."""
        if not self._sensor_cli.wait_for_service(timeout_sec=5.0):
            self.get_logger().warn('/get_sensor_snapshot not available — proceeding without sensor data')
            return json.dumps({'error': 'sensor service unavailable'})
        req    = SensorSnapshot.Request()
        future = self._sensor_cli.call_async(req)
        # Spin until done with timeout
        deadline = time.time() + 5.0
        while not future.done() and time.time() < deadline:
            time.sleep(0.05)
        if not future.done() or future.result() is None:
            return json.dumps({'error': 'sensor snapshot timeout'})
        return future.result().json_data

    def _dispatch_tool_sync(self, tool_name: str, args_json: str) -> str:
        """Call /execute_tool synchronously (blocking)."""
        if not self._tool_cli.wait_for_service(timeout_sec=5.0):
            return json.dumps({'error': 'execute_tool service unavailable'})

        req            = ExecuteTool.Request()
        req.tool_name  = tool_name
        req.args_json  = args_json

        future = self._tool_cli.call_async(req)
        timeout = 180.0 if tool_name == 'explore_and_map' else 60.0
        deadline = time.time() + timeout   # generous timeout for nav/exploration
        while not future.done() and time.time() < deadline:
            time.sleep(0.1)

        if not future.done() or future.result() is None:
            return json.dumps({'error': f'{tool_name} timed out'})

        result = future.result()
        if result.success:
            return result.result_json
        return json.dumps({'error': result.error_message})

    def _publish(self, text: str):
        self.get_logger().info(f'[Brain → /brain/response] {text}')
        self._response_pub.publish(String(data=text))


def main(args=None):
    rclpy.init(args=args)
    node = NvidiaBrainNode()
    # Use MultiThreadedExecutor to allow concurrent service calls inside callbacks
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
