from launch_ros.actions import Node
from os import environ

from launch import LaunchDescription

# Running imu and diff drive controller node with sudo due to needing
# GPIO. This will also pass in relevant environment variables when launching
# the nodes

# This will run on odroid with passwordless sudo
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
            package='imu_adafruit_bno055',
            node_executable='imu_bno055_publisher',
            output='screen',
        ),
        Node(
            prefix=node_prefix,
            package='motor_controls',
            node_executable='diff_drive_controller_node',
            output='screen',
            parameters=[{
                "left_wheel_names": ["left_wheels"],
                "right_wheel_names": ["right_wheels"],
                "write_op_modes": ["motor_controller"],
                "wheel_separation": 0.21,
                "wheels_per_side": 1,  # actually 2, but both are controlled by 1 signal
                "wheel_radius": 0.05,
                "open_loop": True,
                "enable_odom_tf": True,
                "publish_limited_velocity": True,
            }],
        ),
        Node(
            package='tf2_ros',
            node_executable='static_transform_publisher',
            output='screen',
            arguments=['0', '0', '0.03', '0', '0', '0', 'base_link', 'imu_bno055_link']
        )
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
