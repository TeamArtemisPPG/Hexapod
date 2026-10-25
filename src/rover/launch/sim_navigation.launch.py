"""
sim_navigation.launch.py — Gazebo Simulation Navigation, Robot Spawner & RViz IK Hexapod Controller

Features:
  1. Sets GZ_SIM_RESOURCE_PATH for mesh loading (model://rover/meshes/*.stl)
  2. Starts Gazebo Sim with environment obstacles
  3. Spawns hexa robot model via URDF file
  4. Launches robot_state_publisher, ROS-Gazebo bridge & odom TF broadcaster
  5. Launches rviz_hexa_ik controller node for hexapod Inverse Kinematics walking gait
  6. Starts SLAM Toolbox online dynamic mapping (t=0s)
  7. Autostarts Nav2 lifecycle manager (t=15s) once map->odom TF is ready
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, GroupAction,
    IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    pkg_dir = get_package_share_directory('rover')
    models_dir = os.path.dirname(pkg_dir)
    nav2_params = os.path.join(pkg_dir, 'config', 'nav2_params.yaml')
    urdf_path = os.path.join(pkg_dir, 'urdf', 'hexa.urdf')
    world_path = os.path.join(pkg_dir, 'worlds', 'sim_world.sdf')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    params_file = LaunchConfiguration('params_file', default=nav2_params)

    # Set GZ_SIM_RESOURCE_PATH so model://rover/meshes/*.stl resolves cleanly
    gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[models_dir, ':', os.environ.get('GZ_SIM_RESOURCE_PATH', '')]
    )

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use simulation clock for Gazebo navigation')

    declare_params_file = DeclareLaunchArgument(
        'params_file', default_value=nav2_params,
        description='Nav2 params YAML file')

    # Read URDF
    with open(urdf_path, 'r') as f:
        robot_desc = f.read()

    # 1. Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_desc,
            'use_sim_time': use_sim_time,
        }]
    )

    # 2. Gazebo Sim Launch (unpaused by -r)
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch', 'gz_sim.launch.py'
            )
        ),
        launch_arguments={'gz_args': f'-r {world_path}'}.items()
    )

    # 3. Spawn Robot in Gazebo Sim
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-name', 'rover',
            '-file', urdf_path,
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.25'
        ]
    )

    # 4. ROS-Gazebo Bridge (Clock, Scan, Odom, CmdVel, TF)
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
        ]
    )

    # 6. Hexapod IK Locomotion Controller Node (hxp_command tripod gait)
    hxp_ik_node = Node(
        package='rover',
        executable='hxp_cmd',
        name='ik_controller',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # 7. Nav2 lifecycle nodes
    lifecycle_nodes = [
        'controller_server',
        'smoother_server',
        'planner_server',
        'behavior_server',
        'bt_navigator',
        'waypoint_follower',
        'velocity_smoother',
    ]

    remappings = []

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
        SetParameter('use_sim_time', use_sim_time),

        Node(package='nav2_controller', executable='controller_server',
             output='screen', parameters=[configured_params],
             remappings=remappings + [('cmd_vel', 'cmd_vel_nav')]),

        Node(package='nav2_smoother', executable='smoother_server',
             name='smoother_server', output='screen',
             parameters=[configured_params], remappings=remappings),

        Node(package='nav2_planner', executable='planner_server',
             name='planner_server', output='screen',
             parameters=[configured_params], remappings=remappings),

        Node(package='nav2_behaviors', executable='behavior_server',
             name='behavior_server', output='screen',
             parameters=[configured_params],
             remappings=remappings + [('cmd_vel', 'cmd_vel_nav')]),

        Node(package='nav2_bt_navigator', executable='bt_navigator',
             name='bt_navigator', output='screen',
             parameters=[configured_params], remappings=remappings),

        Node(package='nav2_waypoint_follower', executable='waypoint_follower',
             name='waypoint_follower', output='screen',
             parameters=[configured_params], remappings=remappings),

        Node(package='nav2_velocity_smoother', executable='velocity_smoother',
             name='velocity_smoother', output='screen',
             parameters=[configured_params],
             remappings=remappings + [
                 ('cmd_vel', 'cmd_vel_nav'),
                 ('cmd_vel_smoothed', 'cmd_vel'),
             ]),
    ])

    # 8. SLAM Lidar Dynamic Mapping
    mapping = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_dir, 'launch', 'mapping.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'slam_params_file': os.path.join(pkg_dir, 'config', 'slam_toolbox_params.yaml'),
        }.items()
    )

    # 9. Staged Autostart: Nav2 lifecycle manager at t=15s
    configure_nav2 = TimerAction(
        period=15.0,
        actions=[
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_navigation',
                output='screen',
                parameters=[{
                    'autostart': True,
                    'node_names': lifecycle_nodes,
                    'bond_timeout': 4.0,
                    'use_sim_time': use_sim_time,
                }],
            )
        ]
    )

    rviz_config_file = os.path.join(pkg_dir, 'config', 'nav2.rviz')

    # 10. RViz2 for visualization
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    return LaunchDescription([
        gz_resource_path,
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),
        declare_use_sim_time,
        declare_params_file,
        gz_sim,
        spawn_robot,
        robot_state_publisher,
        bridge,
        hxp_ik_node,
        mapping,
        nav2_nodes,
        configure_nav2,
        rviz_node,
    ])
