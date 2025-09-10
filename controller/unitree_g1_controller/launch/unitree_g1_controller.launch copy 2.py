#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def launch_setup(context, *args, **kwargs):
    """Launch setup function"""
    # 获取参数
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context) == 'true'
    
    # 获取包路径
    pkg_share = get_package_share_directory('unitree_g1_description')

    # 控制器配置文件路径
    controller_config = os.path.join(pkg_share, 'config', 'ros2_control','ros2_controllers.yaml')
    
    # ROS2 Control 节点
    controller_manager_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        name='controller_manager',
        output='screen',
        parameters=[
            controller_config,
            {'use_sim_time': use_sim_time}
        ],
        remappings=[
            ('/controller_manager/robot_description', 
             '/robot_description'),
        ]
    )
    
    # Load custom controller
    my_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'my_robot_controller',
            '--controller-manager', 
            '/controller_manager',
        ],
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
        ]
    )
    
    return [
        controller_manager_node,
        # joint_state_broadcaster_spawner,
        my_controller_spawner,
    ]

def generate_launch_description():
    # 声明启动参数
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        OpaqueFunction(function=launch_setup),
    ])