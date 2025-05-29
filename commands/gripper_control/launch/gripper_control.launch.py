import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='gripper_control',
            executable='gripper_control',
            name='gripper_control',
            output='screen',
            parameters=[
                {
                    'trigger_button': 9,
                    'open_position': 0.08,
                    'debounce_time': 1.0,
                    'max_effort': -1.0,
                    'controller_name': 'gripper_controller'
                }
            ],
        )
    ]) 