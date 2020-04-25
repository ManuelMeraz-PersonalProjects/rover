from launch_ros.actions import Node
from os import environ

from launch import LaunchDescription

prefixes = [
    'LD_LIBRARY_PATH', 'AMENT_PREFIX_PATH', 'ROS_VERSION',
    'ROS_PYTHON_VERSION', 'PYTHONPATH', 'PATH', 'ROS_DISTRO'
]

prefixes = [prefix + '=' + environ[prefix] for prefix in prefixes]

node_prefix = 'sudo ' + ' '.join(prefixes)


def generate_launch_description():
    return LaunchDescription([
        Node(
            prefix=node_prefix,
            name='imu_bno055_publisher',
            package='imu_adafruit_bno055',
            node_executable='imu_bno055_publisher',
            output='screen',
        ),
        Node(
            prefix=node_prefix,
            name='diff_drive_controller_node',
            package='motor_controls',
            node_executable='diff_drive_controller_node',
            output='screen',
        #     parameters=[{
        #         'serial_port': '/dev/ttyUSB0',
        #         'serial_baudrate': 115200,
        #         'frame_id': 'laser',
        #         'inverted': False,
        #         'angle_compensate': True,
        #     }],
        ),
        # Node(
        #     node_name='rplidarNodeClient',
        #     package='rplidar_ros',
        #     node_executable='rplidarNodeClient',
        #     output='screen',
        #     parameters=[{
        #         'serial_port': '/dev/ttyUSB0',
        #         'serial_baudrate': 115200,
        #         'frame_id': 'laser',
        #         'inverted': False,
        #         'angle_compensate': True,
        #     }],
        # ),
    ])
