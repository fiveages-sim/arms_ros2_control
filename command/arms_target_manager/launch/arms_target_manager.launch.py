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
    
    control_base_frame_arg = DeclareLaunchArgument(
        'control_base_frame',
        default_value='world',
        description='Target frame ID for pose publishing (marker will transform to this frame)'
    )
    
    marker_fixed_frame_arg = DeclareLaunchArgument(
        'marker_fixed_frame',
        default_value='base_link',
        description='Frame ID where markers are actually created (received current_pose will be transformed to this frame)'
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
            'control_base_frame': LaunchConfiguration('control_base_frame'),
            'marker_fixed_frame': LaunchConfiguration('marker_fixed_frame'),
        }]
    )

    return LaunchDescription([
        topic_prefix_arg,
        dual_arm_mode_arg,
        control_base_frame_arg,
        marker_fixed_frame_arg,
        arms_target_manager_node,
    ])
