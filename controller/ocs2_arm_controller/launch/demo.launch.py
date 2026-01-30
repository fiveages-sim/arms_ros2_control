import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, SetEnvironmentVariable
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Import robot_common_launch utilities
from robot_common_launch import (
    get_robot_package_path,
    get_info_file_name,
    detect_controllers,
    create_controller_spawners,
    get_ros2_control_robot_description,
    prepare_arms_target_manager_parameters,
    parse_launch_mode,
    create_launch_mode_arguments,
)

# All utility functions are now imported from robot_common_launch

def launch_setup(context, *args, **kwargs):
    """Launch setup function using OpaqueFunction"""
    robot_name = context.launch_configurations['robot']
    robot_type = context.launch_configurations.get('type', '')
    hardware = context.launch_configurations.get('hardware', 'mock_components')
    world = context.launch_configurations.get('world', 'dart')

    # 基本参数
    use_sim_time = hardware in ['gz', 'isaac']

    # Launch mode (needed early for planning_robot_state_publisher)
    launch_mode, rviz_only, use_rviz = parse_launch_mode(context)


    # 使用通用的 controller manager launch 文件 (包含 Gazebo 支持、robot_state_publisher 和机器人描述生成)
    controller_manager_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('robot_common_launch'), 'launch'),
            '/controller_manager.launch.py',
        ]),
        launch_arguments=[
            ('robot', robot_name),
            ('type', robot_type),
            ('use_sim_time', str(use_sim_time)),
            ('world', world),
            ('hardware', hardware),  # 传递硬件类型，controller_manager 会根据此参数自动判断是否使用 Gazebo
        ],
    )

    # OCS2 Arm Controller spawner
    ocs2_arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['ocs2_arm_controller'],
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
        ],
    )

    # Detect hand controllers using robot_common_launch (only if gripper is enabled)
    enable_gripper = context.launch_configurations.get('enable_gripper', 'true').lower() == 'true'
    hand_controllers = []
    hand_controller_spawners = []
    
    if enable_gripper:
        # Get ros2_control robot_description to verify joints exist in xacro
        # This uses the same logic as controller_manager.launch.py
        robot_description = get_ros2_control_robot_description(robot_name, robot_type, hardware)
        
        # Detect controllers matching hand/gripper patterns
        # Pass robot_description to verify joints exist in xacro
        hand_controllers = detect_controllers(robot_name, robot_type, ['hand', 'gripper'], robot_description=robot_description)
        hand_controller_spawners = create_controller_spawners(hand_controllers, use_sim_time)

    # Get info file name from controller configuration
    info_file_name = get_info_file_name(robot_name, robot_type)

    # Get robot package path for task file
    robot_pkg_path = get_robot_package_path(robot_name)
    if robot_pkg_path is None:
        print(f"[ERROR] Cannot find robot package path for '{robot_name}'")
        return []

    # OCS2 ArmsTargetManager for interactive pose control (auto-detects dual_arm_mode and frame_id from task.info)
    task_file_path = os.path.join(
        robot_pkg_path,
        "config",
        "ocs2",
        f"{info_file_name}.info"
    )
    print(f"[INFO] Using task file for ArmsTargetManager: {task_file_path}")

    # Extract hand controller names for ArmsTargetManager (if available)
    hand_controller_names_for_target_manager = []
    if enable_gripper and hand_controllers:
        hand_controller_names_for_target_manager = [c['name'] for c in hand_controllers]

    # Prepare parameters using robot_common_launch utility function
    arms_target_manager_parameters = prepare_arms_target_manager_parameters(
        task_file_path=task_file_path,
        config_file_path=None,  # Will auto-detect from task file directory
        hand_controllers=hand_controller_names_for_target_manager if hand_controller_names_for_target_manager else None
    )

    # Create ArmsTargetManager node directly
    ocs2_arms_target_manager = None
    if arms_target_manager_parameters and context.launch_configurations.get('enable_arms_target_manager', 'true').lower() == 'true':
        ocs2_arms_target_manager = Node(
            package='arms_target_manager',
            executable='arms_target_manager_node',
            name='arms_target_manager',
            output='screen',
            parameters=arms_target_manager_parameters,
        )

    # Launch mode: 'full' (default), 'control_only', or 'rviz_only'
    # Note: launch_mode, rviz_only, use_rviz and zenoh_env are already set above

    # RViz for visualization
    rviz_node = None
    if use_rviz:
        # 确定配置文件路径（优先级：机器人description包目录 > 默认配置）
        rviz_config_path = None
        
        # 检查机器人description包目录下是否有demo.rviz配置
        robot_pkg_path = get_robot_package_path(robot_name)
        if robot_pkg_path is not None:
            robot_rviz_config = os.path.join(robot_pkg_path, "config", "rviz", "demo_ocs2.rviz")
            print(f"[INFO] Checking for rviz config in robot description package: {robot_rviz_config}")
            if os.path.exists(robot_rviz_config):
                rviz_config_path = robot_rviz_config
                print(f"[INFO] Using rviz config from robot description package: {rviz_config_path}")
        
        # 如果还没有找到，使用默认配置文件
        if not rviz_config_path:
            rviz_base = os.path.join(
                get_package_share_directory("ocs2_arm_controller"), "config",
            )
            rviz_config_path = os.path.join(rviz_base, "demo.rviz")
            print(f"[INFO] Using default rviz config: {rviz_config_path}")
        
        # Extract hand controller names for GripperControlPanel
        hand_controller_names = []
        if enable_gripper and hand_controllers:
            hand_controller_names = [c['name'] for c in hand_controllers]
        
        # Detect joint controllers for JointControlPanel
        # Always include ocs2_arm_controller
        joint_controller_names = ['ocs2_arm_controller']
        
        # Optionally detect head/body controllers if needed
        # head_controllers = detect_controllers(robot_name, robot_type, ['head', 'body'])
        # joint_controller_names.extend([c['name'] for c in head_controllers])
        
        # Prepare RViz parameters
        rviz_parameters = [{'use_sim_time': use_sim_time}]
        
        # Only add hand_controllers parameter if we have controllers
        if hand_controller_names:
            rviz_parameters.append({'hand_controllers': hand_controller_names})
        
        # Add joint_controllers parameter for JointControlPanel
        rviz_parameters.append({'joint_controllers': joint_controller_names})
        
        rviz_node = Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="log",
            arguments=["-d", rviz_config_path],
            parameters=rviz_parameters,
        )

    # 统一的节点列表 - 不管是否使用 Gazebo，控制器都是一样的
    # robot_state_publisher 和 joint_state_broadcaster 现在由 controller_manager_launch 处理
    nodes = []
    
    # If rviz_only mode, only add rviz (all other nodes should be running on the robot)
    if rviz_only:
        if rviz_node:
            nodes.append(rviz_node)
        # In rviz_only mode, we don't start any other nodes
        # robot_state_publisher, controller_manager, etc. should be running on the robot
    else:
        # Normal mode: add all nodes
        if rviz_node:
            nodes.append(rviz_node)
        nodes.append(controller_manager_launch)
        nodes.append(ocs2_arm_controller_spawner)
    
    # Add ArmsTargetManager node if created (skip in rviz_only mode)
    if not rviz_only and ocs2_arms_target_manager:
        nodes.append(ocs2_arms_target_manager)

    # Add hand controller spawners if any were detected (skip in rviz_only mode)
    if not rviz_only:
        nodes.extend(hand_controller_spawners)

    return nodes


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
        'world', default_value='dart', description='Gz sim World (only used when hardware=gz)'
    )

    enable_arms_target_manager_arg = DeclareLaunchArgument(
        'enable_arms_target_manager',
        default_value='true',
        description='Enable ArmsTargetManager for interactive pose control'
    )

    enable_gripper_arg = DeclareLaunchArgument(
        'enable_gripper',
        default_value='true',
        description='Enable gripper controllers and gripper control panel'
    )

    # Get launch mode arguments from common utilities
    launch_mode_args = create_launch_mode_arguments()

    return LaunchDescription([
        robot_name_arg,
        robot_type_arg,
        hardware_arg,
        world_arg,
        enable_arms_target_manager_arg,
        enable_gripper_arg,
        *launch_mode_args,  # Unpack the list of arguments
        OpaqueFunction(function=launch_setup),
    ])