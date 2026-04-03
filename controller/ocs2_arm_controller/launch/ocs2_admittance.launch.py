import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

from robot_common_launch import get_robot_package_path


def launch_setup(context, *args, **kwargs):
    robot_name = context.launch_configurations.get('robot', 'cr5')
    robot_type = context.launch_configurations.get('type', 'ft-90c')
    hardware = context.launch_configurations.get('hardware', 'real')
    world = context.launch_configurations.get('world', 'dart')
    force_control_mode = context.launch_configurations.get(
        'force_control_mode', 'ros2_control_controllers')
    enable_servoj_stream = context.launch_configurations.get('enable_servoj_stream', 'true')
    allow_servoj_with_force_drag = context.launch_configurations.get(
        'allow_servoj_with_force_drag', 'false')
    enable_gripper_io = context.launch_configurations.get('enable_gripper_io', 'true')
    enable_gripper = context.launch_configurations.get('enable_gripper', 'true')
    enable_arms_target_manager = context.launch_configurations.get(
        'enable_arms_target_manager', 'true')
    enable_force_torque_broadcaster = (
        context.launch_configurations.get('enable_force_torque_broadcaster', 'true').lower() == 'true'
    )
    auto_enable_ft_sensor = (
        context.launch_configurations.get('auto_enable_ft_sensor', 'true').lower() == 'true'
    )
    controller_start_delay_sec = float(
        context.launch_configurations.get('controller_start_delay_sec', '4.5'))
    force_service_prefix = context.launch_configurations.get(
        'force_service_prefix', '/dobot_force_control/srv')
    launch_mode = context.launch_configurations.get('launch_mode', 'full')
    config_name = context.launch_configurations.get(
        'ros2_control_config_name', 'ocs2_admittance.yaml')

    use_sim_time = hardware in ['gz', 'isaac']

    robot_pkg_path = get_robot_package_path(robot_name)
    if robot_pkg_path is None:
        print(f"[ERROR] Cannot find robot description package for '{robot_name}'")
        return []

    ros2_control_config_file = os.path.join(
        robot_pkg_path, 'config', 'ros2_control', config_name)
    if not os.path.exists(ros2_control_config_file):
        print(f"[ERROR] Missing OCS2 admittance config: {ros2_control_config_file}")
        return []

    demo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ocs2_arm_controller'),
                'launch',
                'demo.launch.py',
            )
        ),
        launch_arguments={
            'robot': robot_name,
            'type': robot_type,
            'hardware': hardware,
            'world': world,
            'force_control_mode': force_control_mode,
            'enable_servoj_stream': enable_servoj_stream,
            'allow_servoj_with_force_drag': allow_servoj_with_force_drag,
            'enable_gripper_io': enable_gripper_io,
            'enable_gripper': enable_gripper,
            'enable_arms_target_manager': enable_arms_target_manager,
            'launch_mode': launch_mode,
            'ros2_control_config_file': ros2_control_config_file,
            'spawn_ocs2_arm_controller': 'false',
        }.items(),
    )

    nodes = [demo_launch]

    if enable_force_torque_broadcaster:
        ft_broadcaster_spawner = Node(
            package='controller_manager',
            executable='spawner',
            arguments=[
                'force_torque_sensor_broadcaster',
                '--controller-manager',
                '/controller_manager',
            ],
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
        )
    else:
        ft_broadcaster_spawner = None

    ocs2_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'ocs2_arm_controller',
            '--controller-manager',
            '/controller_manager',
        ],
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    admittance_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'admittance_controller',
            '--controller-manager',
            '/controller_manager',
        ],
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    if hardware == 'real' and auto_enable_ft_sensor:
        nodes.append(
            TimerAction(
                period=2.0,
                actions=[
                    ExecuteProcess(
                        cmd=[
                            'ros2', 'service', 'call',
                            f'{force_service_prefix}/EnableFTSensor',
                            'dobot_force_msgs/srv/EnableFTSensor',
                            '{status: 1}',
                        ],
                        output='screen',
                    )
                ],
            )
        )
        nodes.append(
            TimerAction(
                period=3.5,
                actions=[
                    ExecuteProcess(
                        cmd=[
                            'ros2', 'service', 'call',
                            f'{force_service_prefix}/SixForceHome',
                            'dobot_force_msgs/srv/SixForceHome',
                            '{}',
                        ],
                        output='screen',
                    )
                ],
            )
        )
        delayed_actions = [ocs2_controller_spawner, admittance_controller_spawner]
        if ft_broadcaster_spawner is not None:
            delayed_actions.insert(0, ft_broadcaster_spawner)
        nodes.append(
            TimerAction(
                period=controller_start_delay_sec,
                actions=delayed_actions,
            )
        )
    else:
        if ft_broadcaster_spawner is not None:
            nodes.append(ft_broadcaster_spawner)
        nodes.append(ocs2_controller_spawner)
        nodes.append(admittance_controller_spawner)

    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'robot',
            default_value='cr5',
            description='Robot name (default: cr5)',
        ),
        DeclareLaunchArgument(
            'type',
            default_value='ft-90c',
            description='Robot type/variant for FT sensor setup',
        ),
        DeclareLaunchArgument(
            'hardware',
            default_value='real',
            description='Hardware type: real, mock_components, gz, or isaac',
        ),
        DeclareLaunchArgument(
            'world',
            default_value='dart',
            description='Gazebo world (used only when hardware=gz)',
        ),
        DeclareLaunchArgument(
            'launch_mode',
            default_value='full',
            description='Launch mode forwarded to demo.launch.py',
        ),
        DeclareLaunchArgument(
            'enable_gripper',
            default_value='true',
            description='Enable gripper controller spawning',
        ),
        DeclareLaunchArgument(
            'enable_arms_target_manager',
            default_value='true',
            description='Enable ArmsTargetManager for interactive targets',
        ),
        DeclareLaunchArgument(
            'enable_force_torque_broadcaster',
            default_value='true',
            description='Spawn force_torque_sensor_broadcaster for wrench visualization/debug',
        ),
        DeclareLaunchArgument(
            'auto_enable_ft_sensor',
            default_value='true',
            description='Auto call EnableFTSensor(1) and SixForceHome() on real hardware',
        ),
        DeclareLaunchArgument(
            'controller_start_delay_sec',
            default_value='4.5',
            description='Delay before spawning OCS2/admittance controllers on real hardware after FT zeroing',
        ),
        DeclareLaunchArgument(
            'force_service_prefix',
            default_value='/dobot_force_control/srv',
            description='Force service prefix, e.g. /dobot_force_control/srv',
        ),
        DeclareLaunchArgument(
            'force_control_mode',
            default_value='ros2_control_controllers',
            description='Dobot force mode. OCS2 admittance should use ros2_control_controllers.',
        ),
        DeclareLaunchArgument(
            'enable_servoj_stream',
            default_value='true',
            description='Whether dobot hardware write loop sends ServoJ',
        ),
        DeclareLaunchArgument(
            'allow_servoj_with_force_drag',
            default_value='false',
            description='Keep ServoJ streaming while native ForceDrive drag is active',
        ),
        DeclareLaunchArgument(
            'enable_gripper_io',
            default_value='true',
            description='Whether dobot hardware enables gripper Modbus read/write',
        ),
        DeclareLaunchArgument(
            'ros2_control_config_name',
            default_value='ocs2_admittance.yaml',
            description='ros2_control yaml file name inside <robot>_description/config/ros2_control',
        ),
        OpaqueFunction(function=launch_setup),
    ])
