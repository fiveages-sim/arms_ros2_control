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
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
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
    control_mode_arg = DeclareLaunchArgument(
        'control_mode',
        default_value='ocs2',
        description='Mode: ocs2 / force / auto / blend'
    )
    enable_servoj_stream_arg = DeclareLaunchArgument(
        'enable_servoj_stream',
        default_value='false',
        description='Whether hardware write loop sends ServoJ'
    )
    allow_servoj_with_force_drag_arg = DeclareLaunchArgument(
        'allow_servoj_with_force_drag',
        default_value='false',
        description='Allow ServoJ streaming while ForceDrive drag is active (for blend mode)'
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
    enable_arms_target_manager_arg = DeclareLaunchArgument(
        'enable_arms_target_manager',
        default_value='true',
        description='Enable ArmsTargetManager (interactive marker)'
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
        default_value='0',
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
    wrench_topic_arg = DeclareLaunchArgument(
        'wrench_topic',
        default_value='/force_torque_sensor_broadcaster/wrench',
        description='Wrench topic used by fusion guard in auto mode'
    )
    force_enter_threshold_arg = DeclareLaunchArgument(
        'force_enter_threshold',
        default_value='25.0',
        description='Enter force-drag when |F| exceeds this threshold (N)'
    )
    force_exit_threshold_arg = DeclareLaunchArgument(
        'force_exit_threshold',
        default_value='20.0',
        description='Return to OCS2 when |F| is below this threshold (N)'
    )
    torque_enter_threshold_arg = DeclareLaunchArgument(
        'torque_enter_threshold',
        default_value='3.0',
        description='Enter force-drag when |T| exceeds this threshold (Nm)'
    )
    torque_exit_threshold_arg = DeclareLaunchArgument(
        'torque_exit_threshold',
        default_value='2.0',
        description='Return to OCS2 when |T| is below this threshold (Nm)'
    )
    required_samples_arg = DeclareLaunchArgument(
        'required_samples',
        default_value='3',
        description='Debounce samples required to switch mode'
    )
    min_switch_interval_sec_arg = DeclareLaunchArgument(
        'min_switch_interval_sec',
        default_value='0.8',
        description='Minimum interval between successful mode switches'
    )
    attempt_interval_sec_arg = DeclareLaunchArgument(
        'attempt_interval_sec',
        default_value='1.0',
        description='Minimum interval between failed switch retries'
    )
    hold_settle_ms_arg = DeclareLaunchArgument(
        'hold_settle_ms',
        default_value='100',
        description='Wait time after HOLD before entering drag (ms)'
    )
    ocs2_resume_settle_ms_arg = DeclareLaunchArgument(
        'ocs2_resume_settle_ms',
        default_value='150',
        description='Wait time after StopDrag before returning OCS2 (ms)'
    )
    auto_prepare_on_demand_arg = DeclareLaunchArgument(
        'auto_prepare_on_demand',
        default_value='true',
        description='EnableFTSensor + SixForceHome on first force-drag entry'
    )
    prepare_settle_ms_arg = DeclareLaunchArgument(
        'prepare_settle_ms',
        default_value='500',
        description='Wait after EnableFTSensor before SixForceHome (ms)'
    )
    enable_force_torque_broadcaster_arg = DeclareLaunchArgument(
        'enable_force_torque_broadcaster',
        default_value='true',
        description='Spawn force_torque_sensor_broadcaster in auto mode'
    )

    force_mode_condition = IfCondition(
        PythonExpression(["'", LaunchConfiguration('control_mode'), "' == 'force'"])
    )
    auto_mode_condition = IfCondition(
        PythonExpression(["'", LaunchConfiguration('control_mode'), "' == 'auto'"])
    )
    blend_mode_condition = IfCondition(
        PythonExpression(["'", LaunchConfiguration('control_mode'), "' == 'blend'"])
    )
    ocs2_auto_blend_condition = IfCondition(
        PythonExpression([
            "'", LaunchConfiguration('control_mode'), "' == 'ocs2' or '",
            LaunchConfiguration('control_mode'), "' == 'auto' or '",
            LaunchConfiguration('control_mode'), "' == 'blend'"
        ])
    )
    auto_enter_force_or_blend_condition = IfCondition(
        PythonExpression([
            "('", LaunchConfiguration('control_mode'), "' == 'force' or '",
            LaunchConfiguration('control_mode'), "' == 'blend') and '",
            LaunchConfiguration('auto_enter_drag'), "' == 'true'"
        ])
    )
    servoj_stream_by_mode = PythonExpression(
        ["'true' if '", LaunchConfiguration('control_mode'), "' == 'ocs2' else 'false'"]
    )

    demo_launch_ocs2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ocs2_arm_controller'),
                'launch',
                'demo.launch.py'
            )
        ),
        launch_arguments={
            'robot': LaunchConfiguration('robot'),
            'type': LaunchConfiguration('type'),
            'hardware': LaunchConfiguration('hardware'),
            'world': LaunchConfiguration('world'),
            'enable_gripper': LaunchConfiguration('enable_gripper'),
            'enable_arms_target_manager': LaunchConfiguration('enable_arms_target_manager'),
            'force_control_mode': LaunchConfiguration('force_control_mode'),
            'enable_servoj_stream': LaunchConfiguration('enable_servoj_stream'),
            'allow_servoj_with_force_drag': LaunchConfiguration('allow_servoj_with_force_drag'),
            'enable_gripper_io': LaunchConfiguration('enable_gripper_io'),
            'launch_mode': 'full',
        }.items(),
        condition=ocs2_auto_blend_condition,
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
            'enable_servoj_stream': servoj_stream_by_mode,
            'allow_servoj_with_force_drag': LaunchConfiguration('allow_servoj_with_force_drag'),
            'enable_gripper_io': LaunchConfiguration('enable_gripper_io'),
        }.items(),
        condition=force_mode_condition,
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=['-d', LaunchConfiguration('rviz_config')],
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
            {'hand_controllers': ['hand_controller']},
            {'joint_controllers': ['ocs2_arm_controller']},
        ],
        condition=IfCondition(
            PythonExpression([
                "'", LaunchConfiguration('control_mode'), "' == 'force' and '",
                LaunchConfiguration('enable_rviz'), "' == 'true'"
            ])
        ),
    )

    hand_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['hand_controller'],
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        condition=IfCondition(
            PythonExpression([
                "'", LaunchConfiguration('control_mode'), "' == 'force' and '",
                LaunchConfiguration('enable_gripper'), "' == 'true'"
            ])
        ),
    )
    ft_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['force_torque_sensor_broadcaster'],
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        condition=IfCondition(
            PythonExpression([
                "'", LaunchConfiguration('control_mode'), "' == 'auto' and '",
                LaunchConfiguration('enable_force_torque_broadcaster'), "' == 'true'"
            ])
        ),
    )

    fusion_guard_node = Node(
        package='ocs2_arm_controller',
        executable='force_drag_fusion_guard',
        name='force_drag_fusion_guard',
        output='screen',
        parameters=[{
            'wrench_topic': LaunchConfiguration('wrench_topic'),
            'force_service_prefix': LaunchConfiguration('force_service_prefix'),
            'force_enter_threshold': LaunchConfiguration('force_enter_threshold'),
            'force_exit_threshold': LaunchConfiguration('force_exit_threshold'),
            'torque_enter_threshold': LaunchConfiguration('torque_enter_threshold'),
            'torque_exit_threshold': LaunchConfiguration('torque_exit_threshold'),
            'required_samples': LaunchConfiguration('required_samples'),
            'min_switch_interval_sec': LaunchConfiguration('min_switch_interval_sec'),
            'attempt_interval_sec': LaunchConfiguration('attempt_interval_sec'),
            'hold_settle_ms': LaunchConfiguration('hold_settle_ms'),
            'ocs2_resume_settle_ms': LaunchConfiguration('ocs2_resume_settle_ms'),
            'drag_speed': LaunchConfiguration('drag_speed'),
            'drag_x': LaunchConfiguration('drag_x'),
            'drag_y': LaunchConfiguration('drag_y'),
            'drag_z': LaunchConfiguration('drag_z'),
            'drag_rx': LaunchConfiguration('drag_rx'),
            'drag_ry': LaunchConfiguration('drag_ry'),
            'drag_rz': LaunchConfiguration('drag_rz'),
            'drag_user': LaunchConfiguration('drag_user'),
            'auto_prepare_on_demand': LaunchConfiguration('auto_prepare_on_demand'),
            'prepare_settle_ms': LaunchConfiguration('prepare_settle_ms'),
        }],
        condition=auto_mode_condition,
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

    force_bootstrap_hold = ExecuteProcess(
        cmd=['ros2', 'topic', 'pub', '--once', '/fsm_command', 'std_msgs/msg/Int32', '{data: 2}'],
        output='screen',
        condition=blend_mode_condition,
    )

    auto_enter_drag_group = GroupAction(
        condition=IfCondition(
            PythonExpression([
                "'", LaunchConfiguration('control_mode'), "' == 'force' and '",
                LaunchConfiguration('auto_enter_drag'), "' == 'true'"
            ])
        ),
        actions=[
            TimerAction(period=5.0, actions=[enable_ft]),
            TimerAction(period=6.0, actions=[six_force_home]),
            TimerAction(period=7.2, actions=[enter_drag_mode]),
            TimerAction(period=8.0, actions=[set_drag_speed]),
        ],
    )

    # Blend mode: force bootstrap in strict sequence (no overlap),
    # then user can manually send fsm_command=3 to enter OCS2.
    blend_force_bootstrap_group = GroupAction(
        condition=IfCondition(
            PythonExpression([
                "'", LaunchConfiguration('control_mode'), "' == 'blend' and '",
                LaunchConfiguration('auto_enter_drag'), "' == 'true'"
            ])
        ),
        actions=[
            TimerAction(period=4.5, actions=[force_bootstrap_hold]),
            TimerAction(period=5.0, actions=[enable_ft]),
            RegisterEventHandler(
                OnProcessExit(
                    target_action=enable_ft,
                    on_exit=[TimerAction(period=0.6, actions=[six_force_home])]
                )
            ),
            RegisterEventHandler(
                OnProcessExit(
                    target_action=six_force_home,
                    on_exit=[TimerAction(period=0.8, actions=[enter_drag_mode])]
                )
            ),
            RegisterEventHandler(
                OnProcessExit(
                    target_action=enter_drag_mode,
                    on_exit=[TimerAction(period=0.6, actions=[set_drag_speed])]
                )
            ),
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
        control_mode_arg,
        enable_servoj_stream_arg,
        allow_servoj_with_force_drag_arg,
        enable_gripper_io_arg,
        enable_gripper_arg,
        enable_arms_target_manager_arg,
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
        wrench_topic_arg,
        force_enter_threshold_arg,
        force_exit_threshold_arg,
        torque_enter_threshold_arg,
        torque_exit_threshold_arg,
        required_samples_arg,
        min_switch_interval_sec_arg,
        attempt_interval_sec_arg,
        hold_settle_ms_arg,
        ocs2_resume_settle_ms_arg,
        auto_prepare_on_demand_arg,
        prepare_settle_ms_arg,
        enable_force_torque_broadcaster_arg,
        demo_launch_ocs2,
        controller_manager_launch,
        rviz_node,
        hand_controller_spawner,
        ft_broadcaster_spawner,
        fusion_guard_node,
        auto_enter_drag_group,
        blend_force_bootstrap_group,
        stop_drag_on_shutdown,
    ])
