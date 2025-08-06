import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.actions import IncludeLaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node
from launch.conditions import IfCondition, UnlessCondition
import xacro

def launch_setup(context, *args, **kwargs):
    """Launch setup function using OpaqueFunction"""
    robot_name = context.launch_configurations['robot']
    robot_type = context.launch_configurations.get('type', '')
    hardware = context.launch_configurations.get('hardware', 'mock_components')
    world = context.launch_configurations.get('world', 'empty')
    
    # 构建mappings，只在robot_type有值时才包含
    mappings = {
        'ros2_control_hardware_type': hardware,
    }
    if robot_type and robot_type.strip():
        mappings["type"] = robot_type
    
    # 如果是gazebo模式，添加gazebo相关mappings
    use_gazebo = hardware == 'gz'
    use_sim_time = hardware in ['gz', 'isaac']
    if use_gazebo:
        mappings['gazebo'] = 'true'
    
    # Gazebo相关节点（仅在gazebo模式下创建）
    gazebo = None
    gz_spawn_entity = None
    bridge = None
    
    if use_gazebo:
        # World file (仅在gazebo模式下使用)
        world_path = os.path.join(get_package_share_directory('gz_ros2_control'), 'worlds', world + '.world')
        gazebo = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(get_package_share_directory('ros_gz_sim'), 'launch'),
                '/gz_sim.launch.py',
            ]),
            launch_arguments=[
                ('gz_args', ['-r -v 4 ', world_path])
            ],
        )

        # Spawn robot in Gazebo (仅在gazebo模式下使用)
        gz_spawn_entity = Node(
            package='ros_gz_sim',
            executable='create',
            output='screen',
            arguments=[
                '-topic',
                'robot_description',
                '-name',
                robot_name,
                '-allow_renaming',
                'true',
            ],
        )

        # Bridge for clock (仅在gazebo模式下使用)
        bridge = Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
            output='screen',
        )

    # Robot description
    robot_description_file_path = os.path.join(
        get_package_share_directory(robot_name + "_description"),
        "xacro",
        "ros2_control",
        "robot.xacro"
    )
    
    robot_description_config = xacro.process_file(
        robot_description_file_path,
        mappings=mappings
    )
    
    robot_description = {"robot_description": robot_description_config.toxml()}

    # Robot state publisher
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description],
    )

    # ros2_control using FakeSystem as hardware (仅在非gazebo模式下使用)
    ros2_controllers_path = os.path.join(
        get_package_share_directory(robot_name+"_description"),
        "config",
        "ros2_control",
        "ros2_controllers.yaml",
    )
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            ros2_controllers_path,
            {'use_sim_time': use_sim_time}
        ],
        remappings=[
            ("/controller_manager/robot_description", "/robot_description"),
        ],
        output="screen",
        condition=UnlessCondition(str(use_gazebo)),
    )

    # Controller spawner nodes
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager',
            '/controller_manager',
        ],
        output='screen',
    )

    # OCS2 Arm Controller spawner
    ocs2_arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['ocs2_arm_controller'],
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # Mobile Manipulator Target node for sending target trajectories
    task_file_path = os.path.join(
        get_package_share_directory(robot_name + "_description"),
        "config",
        "ocs2",
        "task.info"
    )
    
    mobile_manipulator_target_node = Node(
        package='ocs2_mobile_manipulator_ros',
        executable='mobile_manipulator_target',
        name='mobile_manipulator_target',
        output='screen',
        parameters=[
            {'taskFile': task_file_path},
            {'use_sim_time': use_sim_time},
            {'enableJoystick': True},  # 默认启用手柄控制
            {'enableAutoPosition': True},  # 默认启用自动位置同步
        ],
        remappings=[
            ('mobile_manipulator_mpc_target', robot_name + '_mpc_target'),
            ('mobile_manipulator_mpc_observation', robot_name + '_mpc_observation'),
            ('mobile_manipulator_end_effector_pose', robot_name + '_end_effector_pose'),
        ],
    )

    # RViz for visualization
    rviz_base = os.path.join(
        get_package_share_directory("ocs2_arm_controller"), "config",
    )
    rviz_full_config = os.path.join(rviz_base, "demo.rviz")
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_full_config],
        parameters=[
            robot_description,
            {'use_sim_time': use_sim_time}
        ],
    )

    # 根据hardware模式返回不同的节点列表
    if use_gazebo:
        # Gazebo模式：使用事件处理器来确保正确的启动顺序
        return [
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=gz_spawn_entity,
                    on_exit=[joint_state_broadcaster_spawner],
                )
            ),
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=joint_state_broadcaster_spawner,
                    on_exit=[ocs2_arm_controller_spawner],
                )
            ),
            bridge,
            gazebo,
            node_robot_state_publisher,
            gz_spawn_entity,
            rviz_node,
            mobile_manipulator_target_node,
        ]
    else:
        # Mock components模式：直接启动所有节点
        return [
            rviz_node,
            node_robot_state_publisher,
            ros2_control_node,
            joint_state_broadcaster_spawner,
            ocs2_arm_controller_spawner,
            mobile_manipulator_target_node,
        ]


def generate_launch_description():
    # Command-line arguments
    robot_name_arg = DeclareLaunchArgument(
        "robot",
        default_value="cr5",
        description="Robot name (arx5, cr5, etc.)"
    )
    
    robot_type_arg = DeclareLaunchArgument(
        "type",
        default_value="", 
        description="Robot type (x5, r5, robotiq85, etc.). Leave empty to not pass type parameter to xacro."
    )

    hardware_arg = DeclareLaunchArgument(
        "hardware",
        default_value="mock_components",
        description="Hardware type: 'gz' for Gazebo simulation, 'isaac' for Isaac simulation, 'mock_components' for mock components"
    )

    world_arg = DeclareLaunchArgument(
        'world', default_value='empty', description='Gz sim World (only used when hardware=gz)'
    )

    return LaunchDescription([
        robot_name_arg,
        robot_type_arg,
        hardware_arg,
        world_arg,
        OpaqueFunction(function=launch_setup),
    ]) 