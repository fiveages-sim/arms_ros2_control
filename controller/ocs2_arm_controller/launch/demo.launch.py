import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition

# Import robot_common_launch utilities
from robot_common_launch import (
    get_robot_package_path,
    get_planning_urdf_path,
    get_info_file_name,
    detect_controllers,
    create_controller_spawners,
    get_ros2_control_robot_description
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

    # Planning robot state publisher for OCS2 planning URDF
    planning_urdf_path = get_planning_urdf_path(robot_name, robot_type)

    planning_robot_state_publisher = None
    if planning_urdf_path is not None:
        try:
            # Read the planning URDF file directly (not through xacro)
            with open(planning_urdf_path, 'r') as urdf_file:
                planning_urdf_content = urdf_file.read()

            planning_robot_description = {"robot_description": planning_urdf_content}
            planning_robot_state_publisher = Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                name='planning_robot_state_publisher',
                output='screen',
                parameters=[planning_robot_description],
                remappings=[
                    ('/tf', '/ocs2_tf'),
                    ('/tf_static', '/ocs2_tf_static'),
                    ('/robot_description', '/ocs2_robot_description'),
                ],
            )
        except Exception as e:
            print(f"[WARN] Failed to create planning robot state publisher: {e}")
    else:
        print(f"[WARN] No planning URDF available for robot '{robot_name}'")

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

    ocs2_arms_target_manager = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('arms_target_manager'), 'launch'),
            '/ocs2_arm_target_manager.launch.py',
        ]),
        launch_arguments=[
            ('task_file', task_file_path),
        ],
        condition=IfCondition(LaunchConfiguration('enable_arms_target_manager'))
    )

    # RViz for visualization
    rviz_base = os.path.join(
        get_package_share_directory("ocs2_arm_controller"), "config",
    )
    rviz_full_config = os.path.join(rviz_base, "demo.rviz")
    
    # Extract hand controller names for GripperControlPanel
    hand_controller_names = []
    if enable_gripper and hand_controllers:
        hand_controller_names = [c['name'] for c in hand_controllers]
    
    # Prepare RViz parameters
    rviz_parameters = [{'use_sim_time': use_sim_time}]
    
    # Only add hand_controllers parameter if we have controllers
    if hand_controller_names:
        rviz_parameters.append({'hand_controllers': hand_controller_names})
    
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_full_config],
        parameters=rviz_parameters,
    )

    # 统一的节点列表 - 不管是否使用 Gazebo，控制器都是一样的
    # robot_state_publisher 和 joint_state_broadcaster 现在由 controller_manager_launch 处理
    nodes = [
        rviz_node,
        controller_manager_launch,
        ocs2_arm_controller_spawner,
        ocs2_arms_target_manager,
    ]

    # Add planning robot state publisher if available
    if planning_robot_state_publisher:
        nodes.append(planning_robot_state_publisher)

    # Add hand controller spawners if any were detected
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

    return LaunchDescription([
        robot_name_arg,
        robot_type_arg,
        hardware_arg,
        world_arg,
        enable_arms_target_manager_arg,
        enable_gripper_arg,
        OpaqueFunction(function=launch_setup),
    ])