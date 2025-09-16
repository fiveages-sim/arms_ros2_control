#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # 声明启动参数
    topic_prefix_arg = DeclareLaunchArgument(
        'topic_prefix',
        default_value='arm_controller',
        description='Topic prefix for pose targets'
    )
    
    dual_arm_mode_arg = DeclareLaunchArgument(
        'dual_arm_mode',
        default_value='false',
        description='Enable dual arm mode'
    )
    
    frame_id_arg = DeclareLaunchArgument(
        'frame_id',
        default_value='world',
        description='Frame ID for the interactive markers'
    )

    # ArmsTargetManager节点
    arms_target_manager_node = Node(
        package='arms_target_manager',
        executable='arms_target_manager_node',
        name='arms_target_manager',
        output='screen',
        parameters=[{
            'topic_prefix': LaunchConfiguration('topic_prefix'),
            'dual_arm_mode': LaunchConfiguration('dual_arm_mode'),
            'frame_id': LaunchConfiguration('frame_id'),
        }]
    )

    return LaunchDescription([
        topic_prefix_arg,
        dual_arm_mode_arg,
        frame_id_arg,
        arms_target_manager_node,
    ])
