#!/usr/bin/env python3
"""
real_navigation.launch.py — Real Robot Hardware Navigation Launch File

Hardware Architecture:
  1. RPLidar A1 (USB Serial -> /dev/ttyUSB0, 115200 baud) publishes /scan in frame 'lidar'
  2. ESP32 Bridge (USB Serial -> /dev/ttyUSB1, 115200 baud) controls 18 servos
  3. Robot State Publisher broadcasts TF for the hexapod (hexa.urdf)
  4. Static TF: odom -> base_footprint (identity)
  5. odom_from_tf calculates dynamic position & velocities from SLAM for Nav2 /odom
  6. hexapod_nav_controller translates Nav2 /cmd_vel into tripod IK walking gaits
  7. SLAM Toolbox builds real-time /map and publishes map -> odom TF
  8. Nav2 Stack plans paths and drives the physical hexapod
  9. RViz2 visualizes map, scan, costmaps, and provides 2D Goal Pose interface
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, EmitEvent, GroupAction,
    LogInfo, RegisterEventHandler, TimerAction
)
from launch.conditions import IfCondition
from launch.events import matches_action
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node, SetParameter
from launch_ros.descriptions import ParameterFile
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    pkg_dir = get_package_share_directory('rover')
    nav2_params_default = os.path.join(pkg_dir, 'config', 'nav2_real_params.yaml')
    slam_params_default = os.path.join(pkg_dir, 'config', 'slam_toolbox_real_params.yaml')
    urdf_path = os.path.join(pkg_dir, 'urdf', 'hexa.urdf')
    rviz_config_path = os.path.join(pkg_dir, 'config', 'nav2.rviz')

    # Auto-detect hardware ports
    import glob
    detected_lidar = '/dev/ttyUSB2' if os.path.exists('/dev/ttyUSB2') else '/dev/ttyUSB0'
    detected_esp = '/dev/ttyUSB1' if os.path.exists('/dev/ttyUSB1') else '/dev/ttyUSB0'
    for p in glob.glob('/dev/serial/by-path/*'):
        target = os.path.realpath(p)
        if '05:00.3' in p:
            detected_lidar = target
        elif '07:00.3' in p:
            detected_esp = target

    # Launch Configurations
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    lidar_port = LaunchConfiguration('lidar_port', default=detected_lidar)
    esp_port = LaunchConfiguration('esp_port', default=detected_esp)
    params_file = LaunchConfiguration('params_file', default=nav2_params_default)
    slam_params_file = LaunchConfiguration('slam_params_file', default=slam_params_default)
    open_rviz = LaunchConfiguration('open_rviz', default='true')

    # Launch Arguments
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulation clock (false for real hardware)'
    )
    declare_lidar_port = DeclareLaunchArgument(
        'lidar_port', default_value=detected_lidar,
        description=f'Serial port for RPLidar (detected: {detected_lidar})'
    )
    declare_esp_port = DeclareLaunchArgument(
        'esp_port', default_value=detected_esp,
        description=f'Serial port for ESP32 hexapod controller (detected: {detected_esp})'
    )
    declare_params_file = DeclareLaunchArgument(
        'params_file', default_value=nav2_params_default,
        description='Nav2 parameters file'
    )
    declare_slam_params_file = DeclareLaunchArgument(
        'slam_params_file', default_value=slam_params_default,
        description='SLAM Toolbox parameters file'
    )
    declare_open_rviz = DeclareLaunchArgument(
        'open_rviz', default_value='true',
        description='Launch RViz2 for visualization'
    )

    # 1. Robot Description (URDF) & Robot State Publisher
    with open(urdf_path, 'r') as f:
        robot_desc = f.read()

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_desc,
            'use_sim_time': False,
        }]
    )

    # 2. RPLidar Node
    rplidar_node = Node(
        package='rplidar_ros',
        executable='rplidar_node',
        name='rplidar_node',
        output='screen',
        parameters=[{
            'serial_port': lidar_port,
            'serial_baudrate': 115200,
            'frame_id': 'lidar',
            'inverted': False,
            'angle_compensate': True,
            'scan_mode': 'Sensitivity',
        }]
    )

    # 3. Static TF: odom -> base_footprint
    static_odom_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_odom_broadcaster',
        arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_footprint'],
        output='screen'
    )

    # 4. Dynamic Odometry publisher from TF
    odom_from_tf_node = Node(
        package='rover',
        executable='odom_from_tf',
        name='odom_from_tf',
        output='screen',
        parameters=[{
            'base_frame': 'base_footprint',
            'odom_frame': 'odom',
            'rate': 20.0
        }]
    )

    # 5. ESP32 Hardware Bridge
    esp_bridge_node = Node(
        package='rover',
        executable='esp_bdg',
        name='esp_bridge',
        output='screen',
        parameters=[{
            'port': esp_port,
            'baud': 115200
        }]
    )

    # 6. Hexapod Locomotion IK Controller (hxp_command: cmd_vel -> Tripod Gait IK -> JointTrajectory)
    hexapod_controller_node = Node(
        package='rover',
        executable='hxp_cmd',
        name='hexapod_controller',
        output='screen'
    )

    # 7. SLAM Toolbox Online Async Mapping
    slam_node = LifecycleNode(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        namespace='',
        parameters=[
            slam_params_file,
            {
                'use_lifecycle_manager': False,
                'use_sim_time': False,
            }
        ]
    )

    configure_slam_event = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(slam_node),
            transition_id=Transition.TRANSITION_CONFIGURE
        )
    )

    activate_slam_event = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=slam_node,
            start_state="configuring",
            goal_state="inactive",
            entities=[
                LogInfo(msg="[real_navigation] Activating slam_toolbox node..."),
                EmitEvent(event=ChangeState(
                    lifecycle_node_matcher=matches_action(slam_node),
                    transition_id=Transition.TRANSITION_ACTIVATE
                ))
            ]
        )
    )

    # 8. Nav2 Nodes
    lifecycle_nodes = [
        'controller_server',
        'smoother_server',
        'planner_server',
        'behavior_server',
        'bt_navigator',
        'waypoint_follower',
        'velocity_smoother',
    ]

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key='',
            param_rewrites={},
            convert_types=True,
        ),
        allow_substs=True,
    )

    nav2_nodes = GroupAction(actions=[
        SetParameter('use_sim_time', False),

        Node(package='nav2_controller', executable='controller_server',
             output='screen', parameters=[configured_params],
             remappings=[('cmd_vel', 'cmd_vel_nav')]),

        Node(package='nav2_smoother', executable='smoother_server',
             name='smoother_server', output='screen',
             parameters=[configured_params]),

        Node(package='nav2_planner', executable='planner_server',
             name='planner_server', output='screen',
             parameters=[configured_params]),

        Node(package='nav2_behaviors', executable='behavior_server',
             name='behavior_server', output='screen',
             parameters=[configured_params],
             remappings=[('cmd_vel', 'cmd_vel_nav')]),

        Node(package='nav2_bt_navigator', executable='bt_navigator',
             name='bt_navigator', output='screen',
             parameters=[configured_params]),

        Node(package='nav2_waypoint_follower', executable='waypoint_follower',
             name='waypoint_follower', output='screen',
             parameters=[configured_params]),

        Node(package='nav2_velocity_smoother', executable='velocity_smoother',
             name='velocity_smoother', output='screen',
             parameters=[configured_params],
             remappings=[
                 ('cmd_vel', 'cmd_vel_nav'),
                 ('cmd_vel_smoothed', 'cmd_vel'),
             ]),
    ])

    # 9. Staged Nav2 Autostart (t=8s to let SLAM warm up on Jetson + SSH latency)
    configure_nav2 = TimerAction(
        period=8.0,
        actions=[
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_navigation',
                output='screen',
                parameters=[{
                    'autostart': True,
                    'node_names': lifecycle_nodes,
                    'bond_timeout': 10.0,  # was 4.0 — prevents killing nodes on slow SSH
                    'use_sim_time': False,
                }],
            )
        ]
    )

    # 10. RViz2 visualization
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        output='screen',
        parameters=[{'use_sim_time': False}],
        condition=IfCondition(open_rviz)
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_lidar_port,
        declare_esp_port,
        declare_params_file,
        declare_slam_params_file,
        declare_open_rviz,
        robot_state_publisher,
        rplidar_node,
        static_odom_tf,
        odom_from_tf_node,
        esp_bridge_node,
        hexapod_controller_node,
        slam_node,
        configure_slam_event,
        activate_slam_event,
        nav2_nodes,
        configure_nav2,
        rviz_node,
    ])
