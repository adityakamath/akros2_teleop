# Copyright (c) 2022 Aditya Kamath
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

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import logging

def generate_launch_description():
    
    teleop_twist_joy_launch_path = PathJoinSubstitution(
        [FindPackageShare('teleop_twist_joy'), 'launch', 'teleop-launch.py'])
    
    ds4_twist_config_path = PathJoinSubstitution(
        [FindPackageShare("akros2_drive"), "config", "ds4_twist_params.yaml"])
    
    twist_mux_topics_config_path = PathJoinSubstitution(
        [FindPackageShare("akros2_drive"), "config", "twist_mux_topics.yaml"])
    
    twist_mux_locks_config_path = PathJoinSubstitution(
        [FindPackageShare("akros2_drive"), "config", "twist_mux_locks.yaml"])
    
    joy_twist_config_dynamic_path = [get_package_share_directory('akros2_drive'), 
                                     '/config/', 
                                     LaunchConfiguration('joy_config'), 
                                     '_twist_config.yaml']
    
    joy_mode_config_dynamic_path = [get_package_share_directory('akros2_drive'), 
                                    '/config/', 
                                    LaunchConfiguration('joy_config'), 
                                    '_mode_config.yaml']
    
    return LaunchDescription([
        DeclareLaunchArgument(
            name='namespace',
            default_value='drive',
            description='Namespace of the system'),
        
        DeclareLaunchArgument(
            name='port_addr',
            default_value='/dev/ttyUSB_TEENSY',
            description='Serial port of the microcontroller'),
        
        DeclareLaunchArgument(
            name='twist',
            default_value='true',
            description='Enable Teleop Twist Joy'),
        
        DeclareLaunchArgument(
            name='feedback',
            default_value='true',
            description='Enable AKROS2 Joy Feedback'),
        
        DeclareLaunchArgument(
            name='agent',
            default_value='true',
            description='Enable Micro-ROS Agent'),
        
        DeclareLaunchArgument(
            name='joy_config',
            default_value='ps4',
            description='Joystick Configuration: ps4, stadia, ps3'),
        
        Node(
            condition=IfCondition(LaunchConfiguration('twist')),
            package='joy',
            executable='joy_node',
            name='joy_node',
            parameters=[{
                'dev': '/dev/input/js0',
                'deadzone': 0.3,
                'autorepeat_rate': 20.0,
            }],
            arguments=["--ros-args", "--log-level", "ERROR"],
        ),
        
        Node(
            condition=IfCondition(LaunchConfiguration('twist')),
            package='teleop_twist_joy',
            executable='teleop_node',
            name='joy_teleop',
            parameters=[joy_twist_config_dynamic_path],
            remappings=[
                ('/cmd_vel', ['/', LaunchConfiguration('namespace'), '/joy_vel']),
            ]),

        Node(
            condition=IfCondition(LaunchConfiguration('feedback')),
            package='akros2_drive',
            executable='joy_feedback',
            name='joy_feedback',
            parameters=[[TextSubstitution(text=os.path.join(get_package_share_directory('akros2_drive'), 'config', '')), 
                                                   LaunchConfiguration('joy_config'), 
                                                   TextSubstitution(text='_mode_config.yaml')]],
            remappings=[
                ('/e_stop', ['/', LaunchConfiguration('namespace'), '/e_stop']),
                ('/mode', ['/', LaunchConfiguration('namespace'), '/mode'])
            ]),
                
        Node(
            package='akros2_drive',
            executable='twist_mixer',
            name='twist_mixer',
            remappings=[
                (['/teleop_vel'], ['/', LaunchConfiguration('namespace'), '/joy_vel']),
                (['/auto_vel'], ['/', LaunchConfiguration('namespace'), '/nav_vel']),
                (['/mix_vel'], ['/', LaunchConfiguration('namespace'), '/mix_vel']),
                (['/mode'], ['/', LaunchConfiguration('namespace'), '/mode']),
            ]),
        
        Node(
            package='twist_mux',
            executable='twist_mux',
            name='twist_mux',
            parameters=[
                twist_mux_topics_config_path,
                twist_mux_locks_config_path,
            ],
            remappings=[
                ('/e_stop',      ['/', LaunchConfiguration('namespace'), '/e_stop']),
                ('/mix_vel',     ['/', LaunchConfiguration('namespace'), '/mix_vel']),
                ('/key_vel',     ['/', LaunchConfiguration('namespace'), '/key_vel']),
                ('/cmd_vel_out', ['/', LaunchConfiguration('namespace'), '/cmd_vel']),
            ]),
        
        Node(
            condition=IfCondition(LaunchConfiguration('agent')),
            package='micro_ros_agent',
            executable='micro_ros_agent',
            name='micro_ros_agent',
            output='screen',
            arguments=['serial', '--dev', LaunchConfiguration("port_addr")]
        )
    ])