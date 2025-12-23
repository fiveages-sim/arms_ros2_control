import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node
import xacro


def launch_setup(context, *args, **kwargs):
    """Launch setup function using OpaqueFunction"""
    hand_name = context.launch_configurations.get('hand', 'linkerhand')
    hand_type = context.launch_configurations.get('type', 'o7')
    direction = context.launch_configurations.get('direction', '1')
    hardware = context.launch_configurations.get('hardware', 'mock_components')
    world = context.launch_configurations.get('world', 'dart')

    # 基本参数
    use_sim_time = hardware in ['gz', 'isaac']

    # 生成 ros2_control robot_description
    # 对于灵巧手，直接处理 hand.xacro 文件
    hand_pkg_name = f"{hand_name}_description"
    hand_pkg_path = get_package_share_directory(hand_pkg_name)
    
    if hand_pkg_path is None:
        print(f"[ERROR] Cannot find package '{hand_pkg_name}'")
        return []
    
    # 查找 ros2_control/hand.xacro 文件
    hand_xacro_path = os.path.join(hand_pkg_path, 'xacro', 'ros2_control', 'hand.xacro')
    
    if not os.path.exists(hand_xacro_path):
        print(f"[ERROR] Hand xacro file not found: {hand_xacro_path}")
        return []
    
    # 构建 xacro mappings
    mappings = {
        'ros2_control_hardware_type': hardware,
    }
    if hand_type and hand_type.strip():
        mappings["type"] = hand_type
    if direction and direction.strip():
        mappings["direction"] = direction
    
    # 如果是 Gazebo 模式，添加 gazebo 映射
    if hardware == 'gz':
        mappings['gazebo'] = 'true'
    
    # 处理 xacro 文件
    try:
        robot_description_config = xacro.process_file(hand_xacro_path, mappings=mappings)
        robot_description = robot_description_config.toxml()
    except Exception as e:
        print(f"[ERROR] Failed to process hand xacro file: {e}")
        return []

    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[
            {
                'publish_frequency': 100.0,
                'use_tf_static': True,
                'robot_description': robot_description
            }
        ],
    )

    # Controller Manager Node
    # 加载控制器配置文件
    controllers_config_path = os.path.join(hand_pkg_path, 'config', 'ros2_control', 'ros2_controllers.yaml')
    
    if not os.path.exists(controllers_config_path):
        print(f"[ERROR] Controllers config file not found: {controllers_config_path}")
        return []
    
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            controllers_config_path,
            {'use_sim_time': use_sim_time},
        ],
        remappings=[
            ("/controller_manager/robot_description", "/robot_description"),
        ],
        output="screen",
    )

    # Joint State Broadcaster spawner
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager',
            '/controller_manager',
        ],
        parameters=[
            {'use_sim_time': use_sim_time},
        ],
        output='screen',
    )

    # Hand Joint Controller spawner
    # 根据 linkerhand_description 的配置，控制器名称是 hand_joint_controller
    hand_joint_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['hand_joint_controller'],
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
        ],
    )

    # RViz for visualization (可选)
    rviz_node = None
    use_rviz = context.launch_configurations.get('use_rviz', 'true').lower() == 'true'
    if use_rviz:
        rviz_base = os.path.join(
            get_package_share_directory("basic_joint_controller"), "config",
        )
        rviz_full_config = os.path.join(rviz_base, "hand.rviz")
        
        # 检查配置文件是否存在，如果不存在则使用默认配置或跳过
        if not os.path.exists(rviz_full_config):
            print(f"[WARN] RViz config file not found: {rviz_full_config}, skipping RViz")
        else:
            # 准备 RViz 参数
            rviz_parameters = [{'use_sim_time': use_sim_time}]
            
            # Add joint_controllers parameter for JointControlPanel
            # 设置默认的控制器名称，这样 joint_control_panel 就能正确找到 topic
            rviz_parameters.append({'joint_controllers': ['hand_joint_controller']})
            print(rviz_parameters)
            
            rviz_node = Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="log",
                arguments=["-d", rviz_full_config],
                parameters=rviz_parameters,
            )

    # 节点列表
    nodes = [
        robot_state_publisher,
        ros2_control_node,
        joint_state_broadcaster_spawner,
        hand_joint_controller_spawner,
    ]

    # 如果启用了 RViz，添加 RViz 节点
    if rviz_node:
        nodes.append(rviz_node)

    return nodes


def generate_launch_description():
    # Command-line arguments
    hand_arg = DeclareLaunchArgument(
        "hand",
        default_value="linkerhand",
        description="Hand name (linkerhand, etc.)"
    )

    type_arg = DeclareLaunchArgument(
        "type",
        default_value="o7",
        description="Hand type (o7, etc.). Leave empty to not pass type parameter to xacro."
    )

    direction_arg = DeclareLaunchArgument(
        "direction",
        default_value="1",
        description="Hand direction (1 for left hand, -1 for right hand)"
    )

    hardware_arg = DeclareLaunchArgument(
        "hardware",
        default_value="mock_components",
        description="Hardware type: 'gz' for Gazebo simulation, 'isaac' for Isaac simulation, 'mock_components' for mock components"
    )

    world_arg = DeclareLaunchArgument(
        'world', default_value='dart', description='Gz sim World (only used when hardware=gz)'
    )

    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Whether to launch RViz visualization'
    )

    return LaunchDescription([
        hand_arg,
        type_arg,
        direction_arg,
        hardware_arg,
        world_arg,
        use_rviz_arg,
        OpaqueFunction(function=launch_setup),
    ])

