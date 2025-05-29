from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='moveit_teleop',
            executable='simple_joystick_servo',
            name='simple_joystick_servo',
            output='screen',
            parameters=[{
                'use_sim_time': False,
            }]
        )
    ]) 