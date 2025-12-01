import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Import robot_common_launch utilities
from robot_common_launch import (
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

    # Detect head controllers using robot_common_launch (only if head is enabled)
    enable_head = context.launch_configurations.get('enable_head', 'true').lower() == 'true'
    head_controllers = []
    head_controller_spawners = []

    if enable_head:
        # Get ros2_control robot_description to verify joints exist in xacro
        # This uses the same logic as controller_manager.launch.py
        robot_description = get_ros2_control_robot_description(robot_name, robot_type, hardware)
        
        # Detect controllers matching head pattern
        # Pass robot_description to verify joints exist in xacro
        head_controllers = detect_controllers(robot_name, robot_type, ['head'], robot_description=robot_description)
        head_controller_spawners = create_controller_spawners(head_controllers, use_sim_time)

    # Detect body controllers using robot_common_launch (only if body is enabled)
    enable_body = context.launch_configurations.get('enable_body', 'true').lower() == 'true'
    body_controllers = []
    body_controller_spawners = []

    if enable_body:
        # Get ros2_control robot_description to verify joints exist in xacro
        # This uses the same logic as controller_manager.launch.py
        robot_description = get_ros2_control_robot_description(robot_name, robot_type, hardware)
        
        # Detect controllers matching body pattern
        # Pass robot_description to verify joints exist in xacro
        body_controllers = detect_controllers(robot_name, robot_type, ['body'], robot_description=robot_description)
        body_controller_spawners = create_controller_spawners(body_controllers, use_sim_time)

    # RViz for visualization (optional)
    rviz_node = None
    use_rviz = context.launch_configurations.get('use_rviz', 'true').lower() == 'true'
    if use_rviz:
        rviz_base = os.path.join(
            get_package_share_directory("basic_joint_controller"), "config",
        )
        rviz_full_config = os.path.join(rviz_base, "demo.rviz")
        
        # Check if config file exists, if not skip RViz
        if not os.path.exists(rviz_full_config):
            print(f"[WARN] RViz config file not found: {rviz_full_config}, skipping RViz")
        else:
            rviz_node = Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="log",
                arguments=["-d", rviz_full_config],
                parameters=[{'use_sim_time': use_sim_time}],
            )

    # 统一的节点列表 - 不管是否使用 Gazebo，控制器都是一样的
    # robot_state_publisher 和 joint_state_broadcaster 现在由 controller_manager_launch 处理
    nodes = [
        controller_manager_launch,
    ]

    # Add head controller spawners if any were detected
    nodes.extend(head_controller_spawners)

    # Add body controller spawners if any were detected
    nodes.extend(body_controller_spawners)

    # Add RViz if enabled
    if rviz_node:
        nodes.append(rviz_node)

    return nodes


def generate_launch_description():
    # Command-line arguments
    robot_name_arg = DeclareLaunchArgument(
        "robot",
        default_value="fiveages_w1",
        description="Robot name (fiveages_w1, etc.)"
    )

    robot_type_arg = DeclareLaunchArgument(
        "type",
        default_value="",
        description="Robot type. Leave empty to not pass type parameter to xacro."
    )

    hardware_arg = DeclareLaunchArgument(
        "hardware",
        default_value="mock_components",
        description="Hardware type: 'gz' for Gazebo simulation, 'isaac' for Isaac simulation, 'mock_components' for mock components"
    )

    world_arg = DeclareLaunchArgument(
        'world', default_value='dart', description='Gz sim World (only used when hardware=gz)'
    )

    enable_head_arg = DeclareLaunchArgument(
        'enable_head',
        default_value='true',
        description='Enable head controllers'
    )

    enable_body_arg = DeclareLaunchArgument(
        'enable_body',
        default_value='true',
        description='Enable body controllers'
    )

    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Whether to launch RViz visualization'
    )

    return LaunchDescription([
        robot_name_arg,
        robot_type_arg,
        hardware_arg,
        world_arg,
        enable_head_arg,
        enable_body_arg,
        use_rviz_arg,
        OpaqueFunction(function=launch_setup),
    ])

