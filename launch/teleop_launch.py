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
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    joy_launch_path = PathJoinSubstitution(
        [FindPackageShare('akros2_teleop'), 'launch', 'joy_launch.py'])

    joy_mode_config_dynamic_path = [get_package_share_directory('akros2_teleop'),
                                    '/config/',
                                    LaunchConfiguration('joy_config'),
                                    '_mode_config.yaml']

    return LaunchDescription([
        DeclareLaunchArgument(
            name='joy_config',
            default_value='steamdeck',
            description='Select Controller: ps4 (PS4/DS4), stadia (Google Stadia), sn30pro (8BitDo SN30 Pro), steamdeck (Valve Steam Deck), none (Disabled)'),

        DeclareLaunchArgument(
            name='executor',
            default_value='True',
            description='If True, run multi-threaded executor. If False, run both nodes separately'),

        Node(
            condition=IfCondition(LaunchConfiguration('executor')),
            package='akros2_teleop',
            executable='teleop_node',
            output='screen',
            parameters=[{'timer_period': 0.02}, joy_mode_config_dynamic_path],
            remappings=[
                ('/teleop_vel', '/joy_vel'),
                ('/auto_vel', '/nav_vel'),
                ('/mix_vel', '/cmd_vel'),
            ]),

        GroupAction(
            condition=UnlessCondition(LaunchConfiguration('executor')),
            actions = [
                Node(
                    package='akros2_teleop',
                    executable='twist_mixer',
                    name='twist_mixer',
                    output='screen',
                    parameters=[{'timer_period': 0.02}],
                    remappings=[('/teleop_vel', '/joy_vel'),
                                ('/auto_vel', '/nav_vel'),
                                ('/mix_vel', '/cmd_vel')]),

                Node(
                    package='akros2_teleop',
                    executable='joy_mode_handler',
                    name='joy_mode_handler',
                    output='screen',
                    parameters=[joy_mode_config_dynamic_path]),
            ]),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(joy_launch_path),
            launch_arguments={'joy_config': LaunchConfiguration('joy_config')}.items()),
    ])
