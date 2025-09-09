import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node
import xacro


def launch_setup(context, *args, **kwargs):
    """Launch setup function using OpaqueFunction"""
    robot_name = context.launch_configurations['robot']
    robot_type = context.launch_configurations.get('type', '')

    # Build mappings, only include when robot_type has a value
    mappings = {
        'ros2_control_hardware_type': 'unitree',
    }
    if robot_type and robot_type.strip():
        mappings["type"] = robot_type

    # Simulation time setting
    use_sim_time = False

    # Robot description
    robot_pkg = robot_name + "_description"
    try:
        robot_pkg_path = get_package_share_directory(robot_pkg)
    except Exception as e:
        print(f"[ERROR] Package '{robot_pkg}' not found: {e}")
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

    # ros2_control using unitree hardware
    ros2_controllers_path = os.path.join(
        robot_pkg_path,
        "config",
        "ros2_control",
        "ros2_controllers.yaml",
    )

    print(f"[INFO] Controller config: {os.path.basename(ros2_controllers_path)}")

    if not os.path.exists(ros2_controllers_path):
        print(f"[ERROR] Controller config not found for '{robot_name}'")
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
            output="screen"
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

    # RViz for visualization
    rviz_base = os.path.join(
        get_package_share_directory("unitree_ros2_control"), "config",
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

    # Return different node lists based on hardware mode
    nodes = [
        rviz_node,
        node_robot_state_publisher,
        joint_state_broadcaster_spawner,
    ]
    # Add ros2_control_node if available
    if ros2_control_node:
        nodes.append(ros2_control_node)
    return nodes


def generate_launch_description():
    # Command-line arguments
    robot_name_arg = DeclareLaunchArgument(
        "robot",
        default_value="unitree_g1",
        description="Robot name (arx5, cr5, etc.)"
    )

    robot_type_arg = DeclareLaunchArgument(
        "type",
        default_value="",
        description="Robot type (x5, r5, robotiq85, etc.). Leave empty to not pass type parameter to xacro."
    )

    return LaunchDescription([
        robot_name_arg,
        robot_type_arg,
        OpaqueFunction(function=launch_setup),
    ])
