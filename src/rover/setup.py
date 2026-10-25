from setuptools import setup
import os
from glob import glob

package_name = 'rover'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/urdf', glob('urdf/*')),
        ('share/' + package_name + '/worlds', glob('worlds/*')),
        (os.path.join('share', package_name, 'meshes'), glob('meshes/*.stl')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'config'), glob('config/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='aanimesh',
    maintainer_email='your@email.com',
    description='Rover robot package',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'teleop = rover.teleop:main',
            'armc = rover.arm_control:main',
            'arm_ik = rover.arm_ik_commander:main',
            'object_detector = rover.object_detector:main',
            'box_tracker = rover.box_tracker:main',
            'tripod_gait = rover.tripod_gait:main',
            'hxp_ik = rover.hexapod_ik_controller:main',
            'imu_stabilizer = rover.imu_stabalizer:main',
            'esp_bdg = rover.esp:main',
            'hxp_cmd = rover.hxp_command:main',
            'cal_servo = rover.test_esp:main',
            'rviz_ik = rover.rviz_hexa_ik:main',
            'hxp_nav_ctrl = rover.hexapod_nav_controller:main',
            'odom_from_tf = rover.odom_from_tf:main',
            # ── Autonomous Brain Stack ─────────────────────────────────────
            'sensor_bridge   = rover.sensor_bridge_node:main',
            'tool_dispatcher = rover.tool_dispatcher_node:main',
            'nvidia_brain    = rover.nvidia_brain_node:main',
            'voice_bridge    = rover.voice_bridge_node:main',
        ],
    },
)

