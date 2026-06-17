import os
import sys

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node

from robot_common_launch import (
    create_launch_mode_arguments,
    create_robot_profile_launch_arguments,
    extract_info_file_name_from_config,
    get_robot_package_path,
    prepare_arms_target_manager_parameters,
)

_launch_dir = os.path.dirname(os.path.abspath(__file__))
if _launch_dir not in sys.path:
    sys.path.insert(0, _launch_dir)
import _ocs2_launch_common as ocs2_common


def launch_setup(context, *args, **kwargs):
    ctx = ocs2_common.build_ocs2_control_context(context)

    planning_robot_name = ocs2_common.resolve_planning_robot_name_from_config(
        ctx.config, "ocs2_arm_controller", ctx.robot_name
    )

    planning_urdf_params = ocs2_common.validate_planning_urdf(
        planning_robot_name,
        ctx.launch_configurations,
        ctx.hardware,
        ctx.profile_path or None,
        planning_scope="arms",
    )
    if planning_urdf_params is None:
        return []

    main_spawner, ocs2_planning_param_file = ocs2_common.create_main_controller_spawner(
        "ocs2_arm_controller", planning_urdf_params
    )

    controller_stack_nodes = ocs2_common.create_controller_stack_nodes(
        ctx,
        context,
        ocs2_planning_param_file=ocs2_planning_param_file,
    )

    enable_gripper = (
        context.launch_configurations.get("enable_gripper", "true").lower() == "true"
    )
    hand_controllers, hand_spawners = ocs2_common.setup_hand_controllers(ctx, enable_gripper)

    enable_body = context.launch_configurations.get("enable_body", "true").lower() == "true"
    body_spawners = []
    joint_controller_names = ["ocs2_arm_controller"]
    if enable_body:
        body_controllers, body_spawners = ocs2_common.setup_body_controllers(ctx, ["body", "head"])
        joint_controller_names.extend(c["name"] for c in body_controllers)

    info_file_name = extract_info_file_name_from_config(ctx.config, launch_mode="split_body")
    task_robot_name = planning_robot_name
    robot_pkg_path = get_robot_package_path(task_robot_name)
    if robot_pkg_path is None:
        print(f"[ERROR] Cannot find robot package path for '{task_robot_name}'")
        return []

    task_file_path = os.path.join(robot_pkg_path, "config", "ocs2", f"{info_file_name}.info")
    print(f"[INFO] Using task file for ArmsTargetManager: {task_file_path}")

    config_file_path = None
    full_body_pkg = get_robot_package_path(ctx.robot_name)
    if full_body_pkg is not None:
        candidate = os.path.join(full_body_pkg, "config", "ocs2", "target_manager.yaml")
        if os.path.exists(candidate):
            config_file_path = candidate
            print(f"[INFO] Using target_manager.yaml from full body robot description: {config_file_path}")
        else:
            print(f"[WARN] target_manager.yaml not found at: {candidate}")

    hand_names = [c["name"] for c in hand_controllers] if enable_gripper else []
    arms_target_manager = None
    arms_params = prepare_arms_target_manager_parameters(
        task_file_path=task_file_path,
        config_file_path=config_file_path,
        hand_controllers=hand_names or None,
    )
    if arms_params and (
        context.launch_configurations.get("enable_arms_target_manager", "true").lower()
        == "true"
    ):
        arms_target_manager = Node(
            package="arms_target_manager",
            executable="arms_target_manager_node",
            name="arms_target_manager",
            output="screen",
            parameters=arms_params + [{"use_sim_time": ctx.use_sim_time}],
        )

    rviz_node = ocs2_common.build_rviz_node(
        ctx,
        ocs2_common.resolve_rviz_config(ctx.robot_name, "splitbody.rviz"),
        hand_names,
        joint_controller_names,
        extra_rviz_parameters=[{"wbc_available": False}],
    )

    return ocs2_common.assemble_nodes(
        ctx,
        rviz_node=rviz_node,
        controller_stack_nodes=controller_stack_nodes,
        main_spawner=main_spawner,
        extra_spawners=hand_spawners + body_spawners,
        optional_nodes=[arms_target_manager] if arms_target_manager else [],
    )


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("robot", default_value="cr5"),
            DeclareLaunchArgument("type", default_value=""),
            DeclareLaunchArgument("hardware", default_value="mock_components"),
            DeclareLaunchArgument("world", default_value="dart"),
            DeclareLaunchArgument("enable_arms_target_manager", default_value="true"),
            DeclareLaunchArgument("enable_gripper", default_value="true"),
            DeclareLaunchArgument("enable_body", default_value="true"),
            DeclareLaunchArgument(
                "use_controller_manager_include",
                default_value="false",
                description="Fallback: use IncludeLaunchDescription for controller_manager",
            ),
            *create_launch_mode_arguments(),
        ]
        + create_robot_profile_launch_arguments()
        + [OpaqueFunction(function=launch_setup)]
    )
