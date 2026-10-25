"""
autonomous.launch.py — Full Autonomous Hexapod Brain Stack

Includes the complete sim_navigation.launch.py stack and adds:
  - sensor_bridge_node  (t=20s — after Nav2 initialises)
  - tool_dispatcher_node (t=21s)
  - nvidia_brain_node   (t=22s — after services are ready)

Usage:
  export NVIDIA_API_KEY="nvapi-xxxxxxxxxxxx"
  ros2 launch rover autonomous.launch.py

Optional overrides:
  ros2 launch rover autonomous.launch.py model:=nvidia/llama-3.1-nemotron-70b-instruct
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('rover')
    brain_config = os.path.join(pkg_dir, 'config', 'brain_config.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    model        = LaunchConfiguration(
        'model', default='meta/llama-3.2-11b-vision-instruct')
    enable_voice = LaunchConfiguration('enable_voice', default='false')

    # ── Args ──────────────────────────────────────────────────────────────────
    declare_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use Gazebo simulation clock')

    declare_model = DeclareLaunchArgument(
        'model', default_value='meta/llama-3.2-11b-vision-instruct',
        description='NVIDIA NIM model name')

    declare_enable_voice = DeclareLaunchArgument(
        'enable_voice', default_value='false',
        description='Enable microphone voice listener node (speech-to-text)')

    # ── Include the full sim navigation stack ─────────────────────────────────
    sim_nav = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_dir, 'launch', 'sim_navigation.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    )

    # ── Sensor Bridge Node (t=20s — wait for Nav2 + map) ─────────────────────
    sensor_bridge = Node(
        package='rover',
        executable='sensor_bridge',
        name='sensor_bridge_node',
        output='screen',
        parameters=[brain_config, {'use_sim_time': use_sim_time}],
    )

    # ── Tool Dispatcher Node (t=21s) ──────────────────────────────────────────
    tool_dispatcher = Node(
        package='rover',
        executable='tool_dispatcher',
        name='tool_dispatcher_node',
        output='screen',
        parameters=[brain_config, {'use_sim_time': use_sim_time}],
    )

    # ── NVIDIA Brain Node (t=22s) ─────────────────────────────────────────────
    brain_node = Node(
        package='rover',
        executable='nvidia_brain',
        name='nvidia_brain_node',
        output='screen',
        parameters=[
            brain_config,
            {
                'use_sim_time':  use_sim_time,
                'model':         model,
            }
        ],
        additional_env={
            # Ensure openai (installed to user dir) is importable
            'PYTHONPATH': (
                '/home/vishal/.local/lib/python3.12/site-packages:'
                + os.environ.get('PYTHONPATH', '')
            ),
        },
    )

    # ── Voice Bridge Node (t=23s, optional via enable_voice:=true) ────────────
    voice_node = Node(
        package='rover',
        executable='voice_bridge',
        name='voice_bridge_node',
        output='screen',
        condition=IfCondition(enable_voice),
        additional_env={
            'PYTHONPATH': (
                '/home/vishal/.local/lib/python3.12/site-packages:'
                + os.environ.get('PYTHONPATH', '')
            ),
        },
    )

    return LaunchDescription([
        declare_sim_time,
        declare_model,
        declare_enable_voice,
        sim_nav,
        TimerAction(period=20.0, actions=[sensor_bridge]),
        TimerAction(period=21.0, actions=[tool_dispatcher]),
        TimerAction(period=22.0, actions=[brain_node]),
        TimerAction(period=23.0, actions=[voice_node]),
    ])
