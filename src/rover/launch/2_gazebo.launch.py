import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
# from ament_index_python.packages import get_package_share_directory
# import os


def generate_launch_description():
    package_dir = get_package_share_directory('rover')
    urdf_path = os.path.join(package_dir, 'urdf', 'hexa.urdf')
    # pkg_share = get_package_share_directory('rover')
    # controllers_file = os.path.join(pkg_share, 'config', 'arm_controllers.yaml')

    with open(urdf_path, 'r') as infp:
        urdf_content = infp.read()

    return LaunchDescription([

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            # name='rsp',
            output='screen',
            parameters=[
                {'robot_description': urdf_content},
                {'use_sim_time': True}   # ✅ FIX
            ]
        ),

        # Node(
        #     package='joint_state_publisher',
        #     executable='joint_state_publisher',
        #     name='rsp',
        #     output='screen',
        #     parameters=[
        #         {'robot_description': urdf_content},
        #         {'use_sim_time': True}   # ✅ FIX
        #     ]
        # ),

        Node(
            package='rviz2',
            executable='rviz2',
            output='screen',
            parameters=[
                {'use_sim_time': True}   # ✅ FIX
            ]
        ),

        ExecuteProcess(
            cmd=[
                'gazebo',
                '--verbose',
                '-s', 'libgazebo_ros_factory.so',
            ],
            output='screen'
        ),


        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            output='screen',
            arguments=[
                '-topic', 'robot_description',
                '-entity', 'rover',
                '-x', '0.0',
                '-y', '0.0',
                '-z', '0.1'
            ]
        ),

        # Node(
        #     package="controller_manager",
        #     executable="ros2_control_node",
        #     parameters=[controllers_file],
        #     output="screen"
        # ),

    ])