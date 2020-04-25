from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from os import environ, path
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
            parameters=[path.join(get_package_share_directory("rover_base"), 'params', 'diff_drive_controller.yaml')],
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
