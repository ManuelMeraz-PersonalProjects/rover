from os import environ, path

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

from launch import LaunchDescription


def generate_launch_description():
    proc_env = environ.copy()
    proc_env['PYTHONUNBUFFERED'] = '1'

    return LaunchDescription([
        Node(
            name='key_teleop',
            package='key_teleop',
            node_executable='key_teleop',
            # output='screen',
            parameters=[
                path.join(get_package_share_directory('rover_base'), 'config',
                          'key_teleop.yaml')
            ],
            env=proc_env
        ),
    ])
