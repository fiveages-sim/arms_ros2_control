import os
import yaml
from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.actions import IncludeLaunchDescription
from launch.actions import RegisterEventHandler
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition, UnlessCondition
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
import xacro


def load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return file.read()
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return yaml.safe_load(file)
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None


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
        .planning_pipelines(pipelines=["ompl", "pilz_industrial_motion_planner"])
        .to_moveit_configs()
    )

    # Get parameters for the Servo node
    servo_yaml = load_yaml(robot_name+"_moveit_config", "config/simulated_config.yaml")
    servo_params = {"moveit_servo": servo_yaml}

    # RViz
    rviz_config_file = os.path.join(
        get_package_share_directory("moveit_common_config"),
        "config",
        "rviz",
        "servo.rviz",
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            {'use_sim_time': use_gazebo}
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

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager-timeout",
            "300",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller", "-c", "/controller_manager"],
    )

    hand_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["hand_controller", "-c", "/controller_manager"],
    )

    # Bridge for clock (仅在gazebo模式下使用)
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen',
        condition=IfCondition(str(use_gazebo)),
    )

    # Launch as much as possible in components
    container = ComposableNodeContainer(
        name="moveit_servo_container",
        namespace="/",
        package="rclcpp_components",
        executable="component_container_mt",
        composable_node_descriptions=[
            ComposableNode(
                package="robot_state_publisher",
                plugin="robot_state_publisher::RobotStatePublisher",
                name="robot_state_publisher",
                parameters=[moveit_config.robot_description],
            ),
            ComposableNode(
                package="moveit_teleop",
                plugin="moveit_teleop::JoyToServoPub",
                name="moveit_joystick_servo_node",
                parameters=[
                    servo_params,
                    {
                        # Gripper control parameters
                        "gripper_open_position": 0.04,
                        "gripper_debounce_time": 1.0,
                    }
                ],
            ),
            ComposableNode(
                package="joy",
                plugin="joy::Joy",
                name="joy_node",
            ),
        ],
        output="screen",
    )

    # Launch a standalone Servo node.
    # As opposed to a node component, this may be necessary (for example) if Servo is running on a different PC
    servo_node = Node(
        package="moveit_servo",
        executable="servo_node_main",
        parameters=[
            servo_params,
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            {'use_sim_time': use_gazebo}
        ],
        output="screen",
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
            gz_spawn_entity,
            rviz_node,
            servo_node,
            container,
        ]
    else:
        # Mock components模式：直接启动所有节点
        return [
            rviz_node,
            ros2_control_node,
            joint_state_broadcaster_spawner,
            arm_controller_spawner,
            hand_controller_spawner,
            servo_node,
            container,
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
        description="Hardware type: 'gz' for Gazebo simulation, 'mock_components' for mock components"
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
