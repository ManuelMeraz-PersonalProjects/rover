import os
import launch
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    camera1_node = Node(
        package='realsense_node',
        node_executable='realsense_node',
        node_namespace="/t265",
        output='screen',
        parameters=[
            os.path.join(get_package_share_directory('rover_base'), 'config',
                         'realsense.yaml')
        ]
    )

    camera2_node = Node(
        package='realsense_node',
        node_executable='realsense_node',
        node_namespace="/d435",
        output='screen',
        parameters=[
            os.path.join(get_package_share_directory('rover_base'), 'config',
                         'realsense.yaml')
        ]
    )

    return launch.LaunchDescription([camera1_node, camera2_node])
