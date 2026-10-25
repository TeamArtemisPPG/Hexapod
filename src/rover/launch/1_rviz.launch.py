import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    package_dir = get_package_share_directory('rover') # Name of the package
    urdf_path = os.path.join(package_dir, 'urdf', 'hexa.urdf') # Name of the file

    with open(urdf_path, 'r') as infp:
        urdf_content = infp.read()

    rviz_config_path = os.path.join(package_dir, 'config', 'view_robot.rviz')

    # Auto-detect hardware ESP32 port
    import glob
    detected_esp = '/dev/ttyUSB0' if os.path.exists('/dev/ttyUSB0') else '/dev/ttyUSB1'
    for p in glob.glob('/dev/serial/by-path/*'):
        target = os.path.realpath(p)
        if '07:00.3' in p:
            detected_esp = target

    from launch.actions import DeclareLaunchArgument
    from launch.substitutions import LaunchConfiguration
    from launch.conditions import IfCondition

    declare_esp_port = DeclareLaunchArgument(
        'esp_port', default_value=detected_esp,
        description=f'Serial port for ESP32 hexapod controller (detected: {detected_esp})'
    )
    declare_hardware = DeclareLaunchArgument(
        'hardware', default_value='true',
        description='Stream joint states to real robot hardware over serial'
    )

    esp_bridge_node = Node(
        package='rover',
        executable='cal_servo',
        name='jointstate_to_esp',
        output='screen',
        parameters=[{
            'port': LaunchConfiguration('esp_port'),
            'baud': 115200
        }],
        condition=IfCondition(LaunchConfiguration('hardware'))
    )

    return LaunchDescription([
        declare_esp_port,
        declare_hardware,
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': urdf_content}]
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen',
        ),
        esp_bridge_node,
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_path],
            output='screen'
        )
    ])
# This launch file starts the robot state publisher, joint state publisher GUI, and RViz for visualization.
# Make sure the URDF file path is correct and the necessary packages are installed.
# To run this launch file, use the command:
# ros2 launch rover 1_rviz.launch.py
