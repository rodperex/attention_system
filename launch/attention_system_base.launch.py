# Copyright 2024 Intelligent Robotics Lab
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

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    pkg_dir = get_package_share_directory('attention_system')
    param_file = os.path.join(pkg_dir, 'config', 'params.yaml')

    ld = LaunchDescription()

    remappings = [
        ('/out_vel', '/cmd_vel')
    ]

    attention_cmd = Node(
        package='attention_system',
        executable='attention_system_base',
        output='screen',
        remappings=remappings,
        parameters=[param_file],
    )
    
    ld.add_action(attention_cmd)
    return ld





