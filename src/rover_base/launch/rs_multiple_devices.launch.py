# Copyright (c) 2019 Intel Corporation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# /* Author: Gary Liu */

import os

import launch
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, ThisLaunchFileDir
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    # config the serial number and base frame id of each camera
    camera1_base_frame_id = LaunchConfiguration('base_frame_id',
                                                default='camera1_link')
    # camera2_base_frame_id = LaunchConfiguration('base_frame_id',
                                                # default='camera2_link')
    camera1_serial_no = LaunchConfiguration('serial_no',
                                            default='939622074301')
    # camera2_serial_no = LaunchConfiguration('serial_no',
                                            # default='943222111242')

    camera1_node = Node(package='realsense_node',
                        node_executable='realsense_node',
                        node_namespace="/camera1",
                        output='screen',
                        parameters=[{
                            'serial_no': camera1_serial_no,
                            'base_frame_id': camera1_base_frame_id
                        }])
    # camera2_node = Node(package='realsense_node',
                        # node_executable='realsense_node',
                        # node_namespace="/camera2",
                        # output='screen',
                        # parameters=[{
                            # 'serial_no': camera2_serial_no,
                            # 'base_frame_id': camera2_base_frame_id
                        # }])
    # return launch.LaunchDescription([camera1_node, camera2_node])
    return launch.LaunchDescription([camera1_node])
