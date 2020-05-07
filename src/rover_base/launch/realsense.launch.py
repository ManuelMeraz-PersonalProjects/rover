import os

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

import launch


def generate_launch_description():
    camera1_node = Node(
        package='realsense_node',
        node_executable='realsense_node',
        namespace="/t265",
        output='screen',
        parameters=[
            os.path.join(get_package_share_directory('rover_base'), 'config',
                         'realsense.yaml')
        ]
    )

    camera2_node = Node(
        package='realsense_node',
        node_executable='realsense_node',
        namespace="/d435",
        output='screen',
        parameters=[
            os.path.join(get_package_share_directory('rover_base'), 'config',
                         'realsense.yaml')
        ]
    )

    return launch.LaunchDescription([camera1_node, camera2_node])
