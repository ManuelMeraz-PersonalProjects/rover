from launch_ros.actions import Node

from launch import LaunchDescription


def generate_launch_description():
    return LaunchDescription([
        Node(
            node_name='imu_bno055_publisher',
            package='imu_adafruit_bno055',
            node_executable='imu_bno055_publisher',
            output='screen',
            launch_prefix='sudo',
        ),
    ])
