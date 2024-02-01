# Copyright (c) 2023 Aditya Kamath
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http:#www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import LaunchConfigurationNotEquals
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    joy_twist_config_dynamic_path = [get_package_share_directory('akros2_teleop'),
                                     '/config/',
                                     LaunchConfiguration('joy_config'),
                                     '_twist_config.yaml']

    return LaunchDescription([
        DeclareLaunchArgument(
            name='joy_config',
            default_value='steamdeck',
            description='Select Controller: ps4 (PS4/DS4), stadia (Google Stadia), sn30pro (8BitDo SN30 Pro), steamdeck (Valve Steam Deck), none (Disabled)'),

        GroupAction(
            condition=LaunchConfigurationNotEquals('joy_config', 'none'),
            actions = [
                Node(
                    package='joy',
                    executable='joy_node',
                    name='joy_node',
                    parameters=[{'dev': '/dev/input/js0',
                                 'deadzone': 0.1,
                                 'autorepeat_rate': 20.0,
                                 'coalesce_interval': 0.01,
                    }],
                    arguments=["--ros-args", "--log-level", "ERROR"]),

                Node(
                    package='teleop_twist_joy',
                    executable='teleop_node',
                    name='joy_teleop',
                    parameters=[joy_twist_config_dynamic_path],
                    remappings=[('/cmd_vel', '/joy_vel')]),
            ]),
    ])
