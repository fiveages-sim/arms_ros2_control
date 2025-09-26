#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get the package directory
    pkg_share = get_package_share_directory('arms_teleop')
    
    # Launch arguments
    joy_dev = LaunchConfiguration('joy_dev')
    joy_dev_arg = DeclareLaunchArgument(
        'joy_dev',
        default_value='/dev/input/js0',
        description='Joystick device file'
    )
    
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
    
    # Joystick teleop node
    joystick_teleop_node = Node(
        package='arms_teleop',
        executable='joystick_teleop',
        name='joystick_teleop',
        output='screen',
        parameters=[
            {
                'joy_dev': joy_dev
            }
        ]
    )
    
    return LaunchDescription([
        joy_dev_arg,
        joy_node,
        joystick_teleop_node
    ]) 