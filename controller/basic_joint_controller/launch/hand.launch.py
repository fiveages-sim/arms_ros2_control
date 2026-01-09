import os
import re

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node
import xacro

# Import robot_common_launch utilities
from robot_common_launch import load_robot_config


def get_default_arg_from_xacro(xacro_file, arg_name):
    """
    从 xacro 文件中读取指定参数的默认值
    
    Args:
        xacro_file (str): xacro 文件路径
        arg_name (str): 参数名称
        
    Returns:
        str: 默认值，如果找不到则返回 None
    """
    try:
        with open(xacro_file, 'r') as f:
            content = f.read()
            # 查找 xacro:arg name="arg_name" default="..." 的模式
            # 支持单引号和双引号
            pattern = rf'xacro:arg\s+name=["\']{re.escape(arg_name)}["\']\s+default=["\']([^"\']+)["\']'
            match = re.search(pattern, content)
            if match:
                return match.group(1)
    except Exception as e:
        print(f"[WARN] Could not read default {arg_name} from {xacro_file}: {e}")
    return None


def launch_setup(context, *args, **kwargs):
    """Launch setup function using OpaqueFunction"""
    hand_name = context.launch_configurations.get('hand', 'linkerhand')
    hand_type = context.launch_configurations.get('type', 'o7')
    direction = context.launch_configurations.get('direction', '1')
    hardware = context.launch_configurations.get('hardware', 'mock_components')
    world = context.launch_configurations.get('world', 'dart')
    serial_port = context.launch_configurations.get('serial_port', '/dev/ttyUSB0')

    # 基本参数
    use_sim_time = hardware in ['gz', 'isaac']

    # 显示手部配置信息
    hand_side = "left" if direction == "1" else "right"
    modbus_id = "0x28" if direction == "1" else "0x27"
    print(f"[INFO] Hand configuration: {hand_side} hand (direction={direction}, Modbus ID={modbus_id})")

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
    
    # Get serial_port from launch configuration (default was already read from xacro in generate_launch_description)
    serial_port = context.launch_configurations.get('serial_port', '/dev/ttyUSB0')
    
    # 构建 xacro mappings
    mappings = {
        'ros2_control_hardware_type': hardware,
    }
    if hand_type and hand_type.strip():
        mappings["type"] = hand_type
    if direction and direction.strip():
        mappings["direction"] = direction
    
    # Pass serial_port and max_speed_ratio to xacro if hardware is real
    if hardware == 'real':
        mappings["serial_port"] = serial_port
        max_speed_ratio = context.launch_configurations.get('max_speed_ratio', '1.0')
        if max_speed_ratio and max_speed_ratio.strip():
            mappings["max_speed_ratio"] = max_speed_ratio
    
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
    # 使用 load_robot_config 根据 type 动态匹配控制器配置文件
    # 支持渐进匹配：如果提供了 type，会依次尝试 {type}.yaml, 缩短后的类型, 最后回退到 ros2_controllers.yaml
    ros2_controllers_config, ros2_controllers_path = load_robot_config(hand_name, "ros2_control", hand_type)
    
    if ros2_controllers_path is None:
        print(f"[ERROR] Controllers config file not found for hand '{hand_name}'")
        return []
    
    # 利用 load_robot_config 已经处理过的结果来判断是否使用了 type-specific 配置
    # load_robot_config 已经处理了回退逻辑，返回的路径就是最终使用的配置文件路径
    # 只要不是默认配置文件名（ros2_controllers.yaml），就说明使用了 type-specific 配置
    config_filename = os.path.basename(ros2_controllers_path)
    default_config_filename = "ros2_controllers.yaml"
    is_type_specific_config = (config_filename != default_config_filename)
    
    # 构建参数列表
    node_parameters = [
        ros2_controllers_path,
        {'use_sim_time': use_sim_time},
        {'robot_description': robot_description},
    ]
    
    # 如果使用了 type-specific 配置，可以在这里添加额外的参数处理逻辑
    # 例如，如果配置文件中没有 type 参数，可以传递 launch 参数中的 type
    if is_type_specific_config:
        print(f"[INFO] Using type-specific config file: {config_filename}")
    else:
        print(f"[INFO] Using default config file: {config_filename}")
    
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=node_parameters,
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
        description="Hand type: 'o7' (7-DOF), 'o6' (6-DOF), or 'l6' (6-DOF). Leave empty to not pass type parameter to xacro."
    )

    direction_arg = DeclareLaunchArgument(
        "direction",
        default_value="1",
        description="Hand direction: '1' for left hand (Modbus ID 0x28), '-1' for right hand (Modbus ID 0x27). Default is left hand (1)."
    )

    hardware_arg = DeclareLaunchArgument(
        "hardware",
        default_value="mock_components",
        description="Hardware type: 'gz' for Gazebo simulation, 'isaac' for Isaac simulation, 'mock_components' for mock components, 'real' for real hardware"
    )

    # Try to read default serial_port from xacro file
    # This is a best-effort attempt - if it fails, we'll use a hardcoded default
    default_serial_port = "/dev/ttyUSB0"  # Fallback default
    try:
        hand_pkg_name = "linkerhand_description"  # Default hand name
        hand_pkg_path = get_package_share_directory(hand_pkg_name)
        if hand_pkg_path:
            hand_xacro_path = os.path.join(hand_pkg_path, 'xacro', 'ros2_control', 'hand.xacro')
            if os.path.exists(hand_xacro_path):
                xacro_default = get_default_arg_from_xacro(hand_xacro_path, 'serial_port')
                if xacro_default:
                    default_serial_port = xacro_default
                    print(f"[INFO] Using default serial_port from xacro: {default_serial_port}")
    except Exception as e:
        print(f"[WARN] Could not read default serial_port from xacro, using fallback: {e}")

    serial_port_arg = DeclareLaunchArgument(
        "serial_port",
        default_value=default_serial_port,
        description=f"Serial port for real hardware (e.g., /dev/ttyUSB0). Only used when hardware=real. Default: {default_serial_port} (read from xacro if available)."
    )

    world_arg = DeclareLaunchArgument(
        'world', default_value='dart', description='Gz sim World (only used when hardware=gz)'
    )

    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Whether to launch RViz visualization'
    )

    max_speed_ratio_arg = DeclareLaunchArgument(
        'max_speed_ratio',
        default_value='1.0',
        description='Maximum speed ratio for joint movement (0.0-1.0). 1.0 = no limiting, 0.5 = 50% of max speed. Only used when hardware=real.'
    )

    return LaunchDescription([
        hand_arg,
        type_arg,
        direction_arg,
        hardware_arg,
        serial_port_arg,
        world_arg,
        use_rviz_arg,
        max_speed_ratio_arg,
        OpaqueFunction(function=launch_setup),
    ])

