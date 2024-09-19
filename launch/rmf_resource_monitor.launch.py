# Copyright (c) 2024 SoftBank Corp.
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
"""Launch file."""

from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description() -> LaunchDescription:
    """Generate launch description.

    Returns:
        LaunchDescription: Launch description.
    """
    param_file_arg = DeclareLaunchArgument(
        'param_file',
        description='Path to the rosparam file to load')
    return LaunchDescription([
        param_file_arg,
        Node(
            package='rmf_resource_monitor',
            executable='resource_monitor_node',
            name='rmf_resource_monitor',
            output='screen',
            parameters=[
                LaunchConfiguration('param_file')
            ]
        ),
    ])
