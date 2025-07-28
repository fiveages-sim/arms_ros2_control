import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.actions import IncludeLaunchDescription
from launch.actions import RegisterEventHandler
from launch.actions import SetEnvironmentVariable
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node
from launch.conditions import IfCondition, UnlessCondition
from moveit_configs_utils import MoveItConfigsBuilder
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
    
    # World file (仅在gazebo模式下使用)
    world_path = os.path.join(get_package_share_directory('moveit_common_config'), 'gazebo', world + '.world')
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch'),
            '/gz_sim.launch.py',
        ]),
        launch_arguments=[
            ('gz_args', ['-r -v 4 ', world_path])
        ],
        condition=IfCondition(str(use_gazebo)),
    )

    # MoveIt配置
    moveit_config = (
        MoveItConfigsBuilder(robot_name)
        .robot_description(
            file_path="xacro/robot.xacro",
            mappings=mappings,
        )
        .robot_description_semantic(file_path="xacro/robot.srdf")
        .trajectory_execution(file_path="config/gripper_moveit_controllers.yaml")
        .planning_pipelines(
            pipelines=["ompl", "chomp", "pilz_industrial_motion_planner"]
        )
        .to_moveit_configs()
    )

    # Robot state publisher
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[moveit_config.robot_description],
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
        condition=IfCondition(str(use_gazebo)),
    )

    # ros2_control using FakeSystem as hardware (仅在非gazebo模式下使用)
    ros2_controllers_path = os.path.join(
        get_package_share_directory(robot_name+"_moveit_config"),
        "config",
        "ros2_controllers.yaml",
    )
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[ros2_controllers_path],
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

    arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['arm_controller'],
        output='screen',
    )

    hand_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['hand_controller'],
        output='screen',
    )

    # Bridge for clock (仅在gazebo模式下使用)
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen',
        condition=IfCondition(str(use_gazebo)),
    )

    # MoveIt move_group node
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict(), {'use_sim_time': use_sim_time}],
        arguments=["--ros-args", "--log-level", "info"],
    )

    # RViz
    rviz_base = os.path.join(
        get_package_share_directory("moveit_common_config"), "config", "rviz"
    )
    rviz_full_config = os.path.join(rviz_base, "demo.rviz")
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_full_config],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
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
                    on_exit=[arm_controller_spawner, hand_controller_spawner],
                )
            ),
            bridge,
            gazebo,
            node_robot_state_publisher,
            gz_spawn_entity,
            move_group_node,
            rviz_node,
        ]
    else:
        # Mock components模式：直接启动所有节点
        return [
            rviz_node,
            node_robot_state_publisher,
            move_group_node,
            ros2_control_node,
            joint_state_broadcaster_spawner,
            arm_controller_spawner,
            hand_controller_spawner,
        ]


def generate_launch_description():
    # Command-line arguments
    robot_name_arg = DeclareLaunchArgument(
        "robot",
        default_value="arx5", 
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