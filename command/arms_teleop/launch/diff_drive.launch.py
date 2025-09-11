#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # Diff drive controller spawner
    diff_drive_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diff_drive_controller'],
        output='screen',
        remappings=[
            ('/diff_drive_controller/cmd_vel', '/cmd_vel')
        ]
    )

    ros_bridge = Node(
        package='rosbridge_server',
        executable='rosbridge_websocket',
        name='rosbridge_websocket',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    rosapi = Node(
        package='rosapi',
        parameters=[
            {'use_sim_time': True},
            {'params_glob': "[*]"}
        ],
        executable='rosapi_node',
        name='rosapi',
    )

    twist_stamper = Node(
        package='twist_stamper',
        executable='twist_stamper',
        name='twist_stamper',
        output='screen',
        remappings=[
            ('cmd_vel_in', '/cmd_vel'),
            ('cmd_vel_out', '/diff_drive_controller/cmd_vel')
        ],
        parameters=[
            {'frame_id': 'base_link'}
        ]
    )

    return LaunchDescription([
        diff_drive_controller_spawner,
        ros_bridge,
        rosapi,
        twist_stamper,
    ])
