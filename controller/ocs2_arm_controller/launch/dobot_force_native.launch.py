import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    ExecuteProcess,
    TimerAction,
    RegisterEventHandler,
    GroupAction,
)
from launch.event_handlers import OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node


def generate_launch_description():
    robot_arg = DeclareLaunchArgument(
        'robot',
        default_value='cr5',
        description='Robot name, e.g. cr5 or cr5_dual'
    )
    type_arg = DeclareLaunchArgument(
        'type',
        default_value='ft-90c',
        description='Robot type, ft-90c is recommended for force sensor setup'
    )
    hardware_arg = DeclareLaunchArgument(
        'hardware',
        default_value='real',
        description='Hardware backend: real / mock_components / gz / isaac'
    )
    world_arg = DeclareLaunchArgument(
        'world',
        default_value='dart',
        description='Gazebo world, used only when hardware=gz'
    )
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )
    force_control_mode_arg = DeclareLaunchArgument(
        'force_control_mode',
        default_value='native_commands',
        description='native_commands or ros2_control_controllers'
    )
    enable_servoj_stream_arg = DeclareLaunchArgument(
        'enable_servoj_stream',
        default_value='false',
        description='Whether hardware write loop sends ServoJ'
    )
    enable_gripper_io_arg = DeclareLaunchArgument(
        'enable_gripper_io',
        default_value='true',
        description='Whether to enable gripper Modbus I/O'
    )
    enable_gripper_arg = DeclareLaunchArgument(
        'enable_gripper',
        default_value='true',
        description='Enable hand_controller spawner'
    )
    enable_rviz_arg = DeclareLaunchArgument(
        'enable_rviz',
        default_value='true',
        description='Start RViz with demo configuration'
    )
    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value=os.path.join(
            get_package_share_directory('ocs2_arm_controller'),
            'config',
            'demo.rviz'
        ),
        description='RViz config file path'
    )
    force_service_prefix_arg = DeclareLaunchArgument(
        'force_service_prefix',
        default_value='/dobot_force_control/srv',
        description='Force service prefix'
    )
    drag_speed_arg = DeclareLaunchArgument(
        'drag_speed',
        default_value='8',
        description='ForceDriveSpeed speed value'
    )
    drag_x_arg = DeclareLaunchArgument('drag_x', default_value='1')
    drag_y_arg = DeclareLaunchArgument('drag_y', default_value='1')
    drag_z_arg = DeclareLaunchArgument('drag_z', default_value='1')
    drag_rx_arg = DeclareLaunchArgument('drag_rx', default_value='1')
    drag_ry_arg = DeclareLaunchArgument('drag_ry', default_value='1')
    drag_rz_arg = DeclareLaunchArgument('drag_rz', default_value='1')
    drag_user_arg = DeclareLaunchArgument(
        'drag_user',
        default_value='-1',
        description='ForceDriveMode user index'
    )
    auto_enter_drag_arg = DeclareLaunchArgument(
        'auto_enter_drag',
        default_value='true',
        description='Automatically call EnableFTSensor/SixForceHome/ForceDriveSpeed/ForceDriveMode'
    )
    auto_stop_drag_on_shutdown_arg = DeclareLaunchArgument(
        'auto_stop_drag_on_shutdown',
        default_value='true',
        description='Automatically call StopDrag on launch shutdown'
    )

    controller_manager_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('robot_common_launch'),
                'launch',
                'controller_manager.launch.py'
            )
        ),
        launch_arguments={
            'robot': LaunchConfiguration('robot'),
            'type': LaunchConfiguration('type'),
            'hardware': LaunchConfiguration('hardware'),
            'world': LaunchConfiguration('world'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'force_control_mode': LaunchConfiguration('force_control_mode'),
            'enable_servoj_stream': LaunchConfiguration('enable_servoj_stream'),
            'enable_gripper_io': LaunchConfiguration('enable_gripper_io'),
        }.items(),
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=['-d', LaunchConfiguration('rviz_config')],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        condition=IfCondition(LaunchConfiguration('enable_rviz')),
    )

    hand_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['hand_controller'],
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        condition=IfCondition(LaunchConfiguration('enable_gripper')),
    )

    enable_ft = ExecuteProcess(
        cmd=[
            'ros2', 'service', 'call',
            [LaunchConfiguration('force_service_prefix'), '/EnableFTSensor'],
            'dobot_force_msgs/srv/EnableFTSensor',
            '{status: 1}',
        ],
        output='screen',
    )

    six_force_home = ExecuteProcess(
        cmd=[
            'ros2', 'service', 'call',
            [LaunchConfiguration('force_service_prefix'), '/SixForceHome'],
            'dobot_force_msgs/srv/SixForceHome',
            '{}',
        ],
        output='screen',
    )

    set_drag_speed = ExecuteProcess(
        cmd=[
            'ros2', 'service', 'call',
            [LaunchConfiguration('force_service_prefix'), '/ForceDriveSpeed'],
            'dobot_force_msgs/srv/ForceDriveSpeed',
            ['{speed: ', LaunchConfiguration('drag_speed'), '}'],
        ],
        output='screen',
    )

    enter_drag_mode = ExecuteProcess(
        cmd=[
            'ros2', 'service', 'call',
            [LaunchConfiguration('force_service_prefix'), '/ForceDriveMode'],
            'dobot_force_msgs/srv/ForceDriveMode',
            [
                '{x: ', LaunchConfiguration('drag_x'),
                ', y: ', LaunchConfiguration('drag_y'),
                ', z: ', LaunchConfiguration('drag_z'),
                ', rx: ', LaunchConfiguration('drag_rx'),
                ', ry: ', LaunchConfiguration('drag_ry'),
                ', rz: ', LaunchConfiguration('drag_rz'),
                ', user: ', LaunchConfiguration('drag_user'),
                '}',
            ],
        ],
        output='screen',
    )

    stop_drag = ExecuteProcess(
        cmd=[
            'ros2', 'service', 'call',
            [LaunchConfiguration('force_service_prefix'), '/StopDrag'],
            'dobot_force_msgs/srv/StopDrag',
            '{}',
        ],
        output='screen',
        condition=IfCondition(LaunchConfiguration('auto_stop_drag_on_shutdown')),
    )

    auto_enter_drag_group = GroupAction(
        condition=IfCondition(LaunchConfiguration('auto_enter_drag')),
        actions=[
            TimerAction(period=5.0, actions=[enable_ft]),
            TimerAction(period=6.0, actions=[six_force_home]),
            TimerAction(period=7.0, actions=[set_drag_speed]),
            TimerAction(period=8.0, actions=[enter_drag_mode]),
        ],
    )

    stop_drag_on_shutdown = RegisterEventHandler(
        OnShutdown(on_shutdown=[stop_drag])
    )

    return LaunchDescription([
        robot_arg,
        type_arg,
        hardware_arg,
        world_arg,
        use_sim_time_arg,
        force_control_mode_arg,
        enable_servoj_stream_arg,
        enable_gripper_io_arg,
        enable_gripper_arg,
        enable_rviz_arg,
        rviz_config_arg,
        force_service_prefix_arg,
        drag_speed_arg,
        drag_x_arg,
        drag_y_arg,
        drag_z_arg,
        drag_rx_arg,
        drag_ry_arg,
        drag_rz_arg,
        drag_user_arg,
        auto_enter_drag_arg,
        auto_stop_drag_on_shutdown_arg,
        controller_manager_launch,
        rviz_node,
        hand_controller_spawner,
        auto_enter_drag_group,
        stop_drag_on_shutdown,
    ])
