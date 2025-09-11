#!/usr/bin/env python3

import os
import yaml
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
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
    """Launch setup function"""
    # Get parameters
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context) == 'true'
    robot_name = LaunchConfiguration('robot').perform(context)
    robot_type = LaunchConfiguration('type').perform(context)
    
    # Get package paths
    ocs2_controller_pkg = get_package_share_directory('ocs2_arm_controller')
    unitree_pkg = get_package_share_directory('unitree_g1_description')
    unitree_control_pkg = get_package_share_directory('unitree_ros2_control')

    # Load robot description from Unitree package
    robot_description_file_path = os.path.join(
        unitree_pkg,
        "xacro",
        "ros2_control",
        "robot.xacro"
    )
    
    # Process XACRO file
    robot_description_config = xacro.process_file(
        robot_description_file_path,
        mappings={'use_sim_time': str(use_sim_time)}
    )
    robot_description = {"robot_description": robot_description_config.toxml()}
    
    # Get controller configurations
    ros2_controllers_path = os.path.join(
        unitree_pkg,
        "config",
        "ros2_control",
        "ros2_controllers.yaml"
    )
    
    # Get task file for OCS2
    task_file_path = os.path.join(
        unitree_pkg,
        "config",
        "ocs2",
        "task.info"
    )

    # Robot state publisher
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': use_sim_time}]
    )

    # ROS2 Control node with Unitree hardware interface
    controller_manager_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            ros2_controllers_path,
            robot_description,
            {'use_sim_time': use_sim_time}
        ],
        output='screen'
    )

    # Spawn joint state broadcaster
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager', 
            '/controller_manager'
        ],
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # Spawn OCS2 G1 controller
    ocs2_g1_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'ocs2_g1_controller',
            '--controller-manager', 
            '/controller_manager'
        ],
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # Mobile Manipulator Target node for sending target trajectories
    mobile_manipulator_target_node = Node(
        package='ocs2_mobile_manipulator_ros',
        executable='mobile_manipulator_target',
        name='mobile_manipulator_target',
        output='screen',
        parameters=[
            {'taskFile': task_file_path},
            {'use_sim_time': use_sim_time},
            {'enableJoystick': True},
            {'enableAutoPosition': True},
            {'enableDynamicFrame': True},
        ],
        remappings=[
            ('mobile_manipulator_mpc_target', robot_name + '_mpc_target'),
            ('mobile_manipulator_mpc_observation', robot_name + '_mpc_observation'),
            ('mobile_manipulator_left_end_effector_pose', robot_name + '_left_end_effector_pose'),
            ('mobile_manipulator_right_end_effector_pose', robot_name + '_right_end_effector_pose'),
        ],
    )

    # RViz visualization
    rviz_config_file = os.path.join(ocs2_controller_pkg, 'config', 'demo.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[
            robot_description,
            {'use_sim_time': use_sim_time}
        ]
    )

    return [
        node_robot_state_publisher,
        controller_manager_node,
        joint_state_broadcaster_spawner,
        ocs2_g1_controller_spawner,
        mobile_manipulator_target_node,
        rviz_node
    ]

def generate_launch_description():
    # Declare launch arguments
    robot_arg = DeclareLaunchArgument(
        'robot',
        default_value='g1',
        description='Robot name'
    )
    
    robot_type_arg = DeclareLaunchArgument(
        'type',
        default_value='',
        description='Robot type/variant'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )

    return LaunchDescription([
        robot_arg,
        robot_type_arg,
        use_sim_time_arg,
        OpaqueFunction(function=launch_setup)
    ])