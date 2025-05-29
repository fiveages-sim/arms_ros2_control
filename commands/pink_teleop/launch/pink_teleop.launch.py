from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
import os
import xacro

package_description = "r5_description"

def process_xacro():
    pkg_path = os.path.join(get_package_share_directory(package_description))
    xacro_file = os.path.join(pkg_path, 'xacro', 'R5.xacro')
    robot_description_config = xacro.process_file(xacro_file)
    return robot_description_config.toxml()

def generate_launch_description():
    
    # Set the URDF path
    robot_description = process_xacro()
    rviz_config_file = os.path.join(get_package_share_directory(package_description), "config", "rviz" ,"visualize.rviz")

    
    # Declare launch arguments
    linear_scale = LaunchConfiguration('linear_scale')
    angular_scale = LaunchConfiguration('angular_scale')
    end_effector_link = LaunchConfiguration('end_effector_link')
    joint_states_topic = LaunchConfiguration('joint_states_topic')
    joint_commands_topic = LaunchConfiguration('joint_commands_topic')
    joy_dev = LaunchConfiguration('joy_dev')
    default_joint_positions = LaunchConfiguration('default_joint_positions')
    
    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument(
            'linear_scale',
            default_value='0.002',
            description='Scale factor for linear velocity'
        ),
        DeclareLaunchArgument(
            'angular_scale',
            default_value='0.01',
            description='Scale factor for angular velocity'
        ),
        DeclareLaunchArgument(
            'end_effector_link',
            default_value='gripper_center',
            description='Name of the end effector link'
        ),
        DeclareLaunchArgument(
            'joint_states_topic',
            default_value='isaac_joint_states',
            description='Topic name for joint states'
        ),
        DeclareLaunchArgument(
            'joint_commands_topic',
            default_value='isaac_joint_commands',
            description='Topic name for joint commands'
        ),
        DeclareLaunchArgument(
            'joy_dev',
            default_value='/dev/input/js0',
            description='Joystick device'
        ),
        DeclareLaunchArgument(
            'default_joint_positions',
            default_value='[0.0, 0.53, 0.073, 0.5]',  # 默认使用robot_wrapper.q0
            description='Default joint positions for the robot'
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz_ocs2',
            output='screen',
            arguments=["-d", rviz_config_file]
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[
                {
                    'publish_frequency': 100.0,
                    'use_tf_static': True,
                    'robot_description': robot_description
                }
            ],
        ),
        # 启动手柄节点
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            parameters=[{
                'dev': joy_dev,
            }],
            output='screen'
        ),
        # Launch the pink teleop node
        Node(
            package='pink_teleop',
            executable='pink_teleop_node',
            name='pink_teleop_node',
            parameters=[{
                'linear_scale': linear_scale,
                'angular_scale': angular_scale,
                'end_effector_link': end_effector_link,
                'joint_states_topic': joint_states_topic,
                'joint_commands_topic': joint_commands_topic,
                'robot_description': robot_description,
                'default_joint_positions': default_joint_positions,
                'use_sim_time': True,
                # 添加夹爪配置
                'gripper_joint_names': ['joint7', 'joint8'],
                'gripper_open_positions': [0.034, 0.034],  # 夹爪打开位置
                'gripper_close_positions': [0.0, 0.0],  # 夹爪关闭位置
            }],
            output='screen'
        )
    ]) 