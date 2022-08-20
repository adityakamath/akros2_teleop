import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import LaunchConfigurationEquals
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def generate_launch_description():
    
    ds4_driver_config_path = PathJoinSubstitution(
        [FindPackageShare("ds4_driver"), "config", "params.yaml"]
    )
    
    ds4_twist_config_path = PathJoinSubstitution(
        [FindPackageShare("akros2_drive"), "config", "ds4_twist_params.yaml"]
    )
    
    twist_mux_topics_config_path = PathJoinSubstitution(
        [FindPackageShare("akros2_drive"), "config", "twist_mux_topics.yaml"]
    )
    
    twist_mux_locks_config_path = PathJoinSubstitution(
        [FindPackageShare("akros2_drive"), "config", "twist_mux_locks.yaml"]
    )
    
    return LaunchDescription([
        DeclareLaunchArgument(
            name='robot_namespace',
            default_value='akros2',
            description='Namespace of the robot'
        ),
        
        DeclareLaunchArgument(
            name='serial_port_addr',
            default_value='/dev/ttyUSB_TEENSY',
            description='Serial port of the microcontroller'
        ),
        
        DeclareLaunchArgument(
            name='ps4_addr',
            default_value='84:30:95:2C:67:7C',
            description='MAC address of the PS4 controller'
        ),
        
        DeclareLaunchArgument(
            name='standalone', 
            default_value='true',
            description='Used for testing this launch file by itself'
        ),
        
        Node(
            package='ds4_driver',
            executable='ds4_driver_node.py',
            name='ds4_driver',
            parameters=[
                {'device_addr': LaunchConfiguration('ps4_addr')},
                {'use_standard_msgs': 'false'},
                {'autorepeat_rate': 0.0},
                ds4_driver_config_path,
            ],
        ),
        Node(
            package='ds4_driver',
            executable='ds4_twist_node.py',
            name='ds4_twist',
            parameters=[
                {'stamped': 'false'},
                ds4_twist_config_path,
            ],
            remappings=[
                ('/cmd_vel', ['/', LaunchConfiguration('robot_namespace'), '/joy_vel']),
            ]
        ),
        Node(
            package='akros2_drive',
            executable='ds4_feedback',
            name='ds4_feedback',
            remappings=[
                ('/mode',   ['/', LaunchConfiguration('robot_namespace'), '/mode']),
                ('/e_stop', ['/', LaunchConfiguration('robot_namespace'), '/e_stop']),
            ]
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='ds4_to_imu',
            arguments=['0', '0.05', '-0.01', '-1.5707', '0', '1.5707', 'ds4', 'ds4_imu']
        ),
        Node(
            package='akros2_drive',
            executable='twist_mixer',
            name='twist_mixer',
            remappings=[
                ('/mode',       ['/', LaunchConfiguration('robot_namespace'), '/mode']),
                ('/teleop_vel', ['/', LaunchConfiguration('robot_namespace'), '/joy_vel']),
                ('/auto_vel',   ['/', LaunchConfiguration('robot_namespace'), '/nav_vel']),
                ('/mix_vel',    ['/', LaunchConfiguration('robot_namespace'), '/mix_vel']),
            ]
        ),
        Node(
            package='twist_mux',
            executable='twist_mux',
            name='twist_mux',
            parameters=[
                twist_mux_topics_config_path,
                twist_mux_locks_config_path,
            ],
            remappings=[
                ('/e_stop',      ['/', LaunchConfiguration('robot_namespace'), '/e_stop']),
                ('/mix_vel',     ['/', LaunchConfiguration('robot_namespace'), '/mix_vel']),
                ('/key_vel',     ['/', LaunchConfiguration('robot_namespace'), '/key_vel']),
                ('/cmd_vel_out', ['/', LaunchConfiguration('robot_namespace'), '/cmd_vel']),
            ]
        ),
        Node(
            condition=LaunchConfigurationEquals('standalone', 'true'),
            package='micro_ros_agent',
            executable='micro_ros_agent',
            name='micro_ros_agent',
            output='screen',
            arguments=['serial', '--dev', LaunchConfiguration("serial_port_addr")]
        )
    ])