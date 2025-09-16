import os
import yaml
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

def get_robot_paths(robot_name):
    """Get common robot-related paths"""
    robot_pkg = robot_name + "_description"
    try:
        robot_pkg_path = get_package_share_directory(robot_pkg)
        return robot_pkg_path
    except Exception as e:
        print(f"[ERROR] Failed to get package path for '{robot_pkg}': {e}")
        return None

def get_robot_config(robot_name, robot_type=""):
    """Get robot configuration from ROS2 controller configuration file"""
    robot_pkg_path = get_robot_paths(robot_name)
    if robot_pkg_path is None:
        return None, None
        
    try:
        # Try type-specific config file first, fallback to default
        config_file = f"{robot_type}.yaml" if robot_type and robot_type.strip() else "ros2_controllers.yaml"
        config_path = os.path.join(robot_pkg_path, "config", "ros2_control", config_file)
        
        if not os.path.exists(config_path):
            config_file = "ros2_controllers.yaml"
            config_path = os.path.join(robot_pkg_path, "config", "ros2_control", config_file)
            print(f"[INFO] Type-specific config not found, using default: {config_file}")
        else:
            print(f"[INFO] Using config file: {config_file}")
            
        print(f"[INFO] Reading controller config from: {config_path}")
        
        with open(config_path, 'r') as file:
            config = yaml.safe_load(file)
            
        return config, config_path
        
    except FileNotFoundError:
        print(f"[WARN] Controller config file not found for robot '{robot_name}'")
        return None, None
    except yaml.YAMLError as e:
        print(f"[ERROR] Failed to parse YAML config for robot '{robot_name}': {e}")
        return None, None
    except Exception as e:
        print(f"[ERROR] Unexpected error reading config for robot '{robot_name}': {e}")
        return None, None

def get_info_file_name(robot_name, robot_type=""):
    """Get info_file_name from ROS2 controller configuration, fallback to 'task'"""
    config, _ = get_robot_config(robot_name, robot_type)
    
    if config is None:
        print(f"[WARN] Using default info_file_name: 'task' for robot '{robot_name}'")
        return 'task'
    
    try:
        # Extract info_file_name from ocs2_arm_controller parameters
        info_file_name = config.get('ocs2_arm_controller', {}).get('ros__parameters', {}).get('info_file_name', 'task')
        print(f"[INFO] Found info_file_name: '{info_file_name}' for robot '{robot_name}'")
        return info_file_name
    except KeyError as e:
        print(f"[WARN] Key error in config for robot '{robot_name}': {e}, using default 'task'")
        return 'task'

def detect_hand_controllers(robot_name, robot_type=""):
    """Detect hand controllers from ROS2 controller configuration"""
    config, _ = get_robot_config(robot_name, robot_type)
    
    if config is None:
        print(f"[WARN] No hand controllers will be loaded for robot '{robot_name}'")
        return []
    
    hand_controllers = []
    
    # Check controller_manager section for hand controllers
    controller_manager = config.get('controller_manager', {}).get('ros__parameters', {})
    
    for controller_name, controller_config in controller_manager.items():
        if 'hand' in controller_name.lower() or 'gripper' in controller_name.lower():
            # Extract controller type and parameters
            controller_type = controller_config.get('type', '')
            hand_controllers.append({
                'name': controller_name,
                'type': controller_type,
                'config': controller_config
            })
            print(f"[INFO] Detected hand controller: {controller_name} ({controller_type})")
    
    # Also check for hand controller parameter sections
    for section_name, section_config in config.items():
        if 'hand' in section_name.lower() or 'gripper' in section_name.lower():
            if section_name not in [c['name'] for c in hand_controllers]:
                hand_controllers.append({
                    'name': section_name,
                    'type': 'unknown',
                    'config': section_config
                })
                print(f"[INFO] Detected hand controller section: {section_name}")
    
    print(f"[INFO] Total hand controllers detected: {len(hand_controllers)}")
    return hand_controllers

def create_hand_controller_spawners(hand_controllers, use_sim_time=False):
    """Create spawner nodes for hand controllers"""
    spawners = []
    
    for controller in hand_controllers:
        controller_name = controller['name']
        
        print(f"[INFO] Creating spawner for hand controller: {controller_name}")
        
        spawner = Node(
            package='controller_manager',
            executable='spawner',
            arguments=[controller_name],
            output='screen',
            parameters=[
                {'use_sim_time': use_sim_time},
            ],
        )
        
        spawners.append(spawner)
    
    return spawners

def get_planning_urdf_path(robot_name, robot_type):
    """Get planning URDF file path based on robot type, similar to CtrlComponent logic"""
    robot_pkg_path = get_robot_paths(robot_name)
    if robot_pkg_path is None:
        return None
    
    if robot_type and robot_type.strip():
        # Try type-specific URDF first
        robot_identifier = robot_name + "_" + robot_type
        type_specific_urdf = os.path.join(robot_pkg_path, "urdf", robot_identifier + ".urdf")
        
        # Check if type-specific URDF exists
        if os.path.exists(type_specific_urdf):
            print(f"[INFO] Using type-specific planning URDF: {type_specific_urdf}")
            return type_specific_urdf
        else:
            # Fallback to default URDF if type-specific doesn't exist
            default_urdf = os.path.join(robot_pkg_path, "urdf", robot_name + ".urdf")
            print(f"[WARN] Type-specific planning URDF not found: {type_specific_urdf}, falling back to default: {default_urdf}")
            return default_urdf
    else:
        # Use default URDF
        default_urdf = os.path.join(robot_pkg_path, "urdf", robot_name + ".urdf")
        print(f"[INFO] Using default planning URDF: {default_urdf}")
        return default_urdf

def launch_setup(context, *args, **kwargs):
    """Launch setup function using OpaqueFunction"""
    robot_name = context.launch_configurations['robot']
    robot_type = context.launch_configurations.get('type', '')
    hardware = context.launch_configurations.get('hardware', 'mock_components')
    world = context.launch_configurations.get('world', 'empty')
    
    # Build mappings, only include when robot_type has a value
    mappings = {
        'ros2_control_hardware_type': hardware,
    }
    if robot_type and robot_type.strip():
        mappings["type"] = robot_type
    
    # If in gazebo mode, add gazebo-related mappings
    use_gazebo = hardware == 'gz'
    use_sim_time = hardware in ['gz', 'isaac']
    if use_gazebo:
        mappings['gazebo'] = 'true'
    
    # Gazebo-related nodes (only created in gazebo mode)
    gazebo = None
    gz_spawn_entity = None
    bridge = None
    
    if use_gazebo:
        # World file (only used in gazebo mode)
        world_path = os.path.join(get_package_share_directory('ocs2_arm_controller'), 'worlds', world + '.sdf')
        gazebo = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(get_package_share_directory('ros_gz_sim'), 'launch'),
                '/gz_sim.launch.py',
            ]),
            launch_arguments=[
                ('gz_args', ['-r -v 4 ', world_path])
            ],
        )

        # Spawn robot in Gazebo (only used in gazebo mode)
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

        # Bridge for clock (only used in gazebo mode)
        bridge = Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
            output='screen',
        )

    # Robot description
    robot_pkg_path = get_robot_paths(robot_name)
    if robot_pkg_path is None:
        print(f"[ERROR] Cannot create robot description without package path for robot '{robot_name}'")
        return []
        
    robot_description_file_path = os.path.join(
        robot_pkg_path,
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
        parameters=[robot_description, {'use_sim_time': use_sim_time}],
    )

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
            print(f"[INFO] Planning robot state publisher created for: {planning_urdf_path}")
        except Exception as e:
            print(f"[WARN] Failed to create planning robot state publisher: {e}")
    else:
        print(f"[WARN] No planning URDF available for robot '{robot_name}'")

    # ros2_control using FakeSystem as hardware (only used in non-gazebo mode)
    _, ros2_controllers_path = get_robot_config(robot_name, robot_type)
    if ros2_controllers_path is None:
        print(f"[ERROR] Cannot create ros2_control_node without controller config for robot '{robot_name}'")
        ros2_control_node = None
    else:
        ros2_control_node = Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[
                ros2_controllers_path,
                {'use_sim_time': use_sim_time},
                # Pass robot_type parameter to the controller if specified
                {'robot_type': robot_type} if robot_type and robot_type.strip() else {}
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
        parameters=[
            {'use_sim_time': use_sim_time},
        ],
        output='screen',
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

    # Get info file name from controller configuration
    info_file_name = get_info_file_name(robot_name, robot_type)
    
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
            ('robot', robot_name),
            ('task_file', task_file_path),
        ],
        condition=IfCondition(LaunchConfiguration('enable_arms_target_manager'))
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

    # Detect hand controllers
    hand_controllers = detect_hand_controllers(robot_name, robot_type)
    hand_controller_spawners = create_hand_controller_spawners(hand_controllers, use_sim_time)

    # Return different node lists based on hardware mode
    if use_gazebo:
        # Gazebo mode: use event handlers to ensure correct startup order
        nodes = [
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
            # Add hand controller spawners after arm controller if any were detected
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=ocs2_arm_controller_spawner,
                    on_exit=hand_controller_spawners,
                )
            ) if hand_controller_spawners else None,
            bridge,
            gazebo,
            node_robot_state_publisher,
            gz_spawn_entity,
            rviz_node,
            ocs2_arms_target_manager,
        ]
        # Add planning robot state publisher if available
        if planning_robot_state_publisher:
            nodes.append(planning_robot_state_publisher)
        # Filter out None values from event handlers
        nodes = [node for node in nodes if node is not None]
        return nodes
    else:
        # Mock components mode: start all nodes directly
        nodes = [
            rviz_node,
            node_robot_state_publisher,
            joint_state_broadcaster_spawner,
            ocs2_arm_controller_spawner,
            ocs2_arms_target_manager,
        ]
        # Add ros2_control_node if available
        if ros2_control_node:
            nodes.append(ros2_control_node)
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
        'world', default_value='empty', description='Gz sim World (only used when hardware=gz)'
    )
    
    enable_arms_target_manager_arg = DeclareLaunchArgument(
        'enable_arms_target_manager',
        default_value='true',
        description='Enable ArmsTargetManager for interactive pose control'
    )

    return LaunchDescription([
        robot_name_arg,
        robot_type_arg,
        hardware_arg,
        world_arg,
        enable_arms_target_manager_arg,
        OpaqueFunction(function=launch_setup),
    ]) 