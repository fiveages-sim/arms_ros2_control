#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Launch arguments
    joy_dev = LaunchConfiguration('joy_dev')
    joy_dev_arg = DeclareLaunchArgument(
        'joy_dev',
        default_value='/dev/input/js0',
        description='Joystick device file'
    )
    
    # Config file name argument (without .yaml extension)
    config = LaunchConfiguration('config')
    config_arg = DeclareLaunchArgument(
        'config',
        default_value='default',
        description='Name of joystick configuration file (without .yaml extension)'
    )
    
    # Build full config file path with automatic .yaml extension
    config_file = PathJoinSubstitution([
        FindPackageShare('arms_teleop'),
        'config',
        [config, TextSubstitution(text='.yaml')]
    ])
    
    # Joy node to read joystick input
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen',
        parameters=[{
            'device_id': 0,
            'deadzone': 0.05,
            'autorepeat_rate': 20.0
        }]
    )
    
    # Joystick teleop node with config file
    joystick_teleop_node = Node(
        package='arms_teleop',
        executable='joystick_teleop',
        name='joystick_teleop',
        output='screen',
        parameters=[config_file]
    )
    
    return LaunchDescription([
        joy_dev_arg,
        config_arg,
        joy_node,
        joystick_teleop_node
    ]) 