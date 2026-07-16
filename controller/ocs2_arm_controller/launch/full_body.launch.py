import os
import sys
import tempfile

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node

from robot_common_launch import (
    create_launch_mode_arguments,
    create_robot_profile_launch_arguments,
    extract_info_file_name_from_config,
    get_robot_package_path,
    parse_task_info,
    prepare_arms_target_manager_parameters,
)

_launch_dir = os.path.dirname(os.path.abspath(__file__))
if _launch_dir not in sys.path:
    sys.path.insert(0, _launch_dir)
import _ocs2_launch_common as ocs2_common


def launch_setup(context, *args, **kwargs):
    ctx = ocs2_common.build_ocs2_control_context(context)

    planning_robot_name = ocs2_common.resolve_planning_robot_name_from_config(
        ctx.config, "ocs2_wbc_controller", ctx.robot_name
    )

    wbc_available = False
    wbc_controller_type = ""
    if ctx.config:
        try:
            cm_params = ctx.config.get("controller_manager", {}).get("ros__parameters", {})
            wbc_controller_cfg = cm_params.get("ocs2_wbc_controller", {})
            wbc_controller_type = wbc_controller_cfg.get("type", "")
            wbc_available = wbc_controller_type == "ocs2_wbc_controller/Ocs2WbcController"
            print(
                f"[INFO] ocs2_wbc_controller.type = {wbc_controller_type}, "
                f"wbc_available = {wbc_available}"
            )
        except Exception as exc:
            print(f"[WARN] Failed to parse ocs2_wbc_controller.type from config: {exc}")

    planning_urdf_params = ocs2_common.validate_planning_urdf(
        planning_robot_name,
        ctx.launch_configurations,
        ctx.hardware,
        ctx.profile_path or None,
        planning_scope="full",
    )
    if planning_urdf_params is None:
        return []

    main_spawner, ocs2_planning_param_file = ocs2_common.create_main_controller_spawner(
        "ocs2_wbc_controller", planning_urdf_params
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
    hand_names = [c["name"] for c in hand_controllers] if enable_gripper else []

    ft_controllers, ft_spawners = ocs2_common.setup_ft_broadcasters(ctx)

    body_spawners = []
    joint_controller_names = ["ocs2_wbc_controller"]
    is_arm_controller_type = wbc_controller_type == "ocs2_arm_controller/Ocs2ArmController"
    if is_arm_controller_type:
        head_controllers, body_spawners = ocs2_common.setup_body_controllers(ctx, ["head"])
        if head_controllers:
            joint_controller_names.extend(c["name"] for c in head_controllers)
            print(
                f"[INFO] ARM type detected in full_body: spawning head controllers "
                f"{[c['name'] for c in head_controllers]}"
            )
        else:
            print("[WARN] ARM type detected in full_body but no head controller found to spawn")

    if hand_names:
        joint_controller_names.extend(hand_names)

    info_file_name = extract_info_file_name_from_config(ctx.config, launch_mode="full_body")
    task_robot_name = planning_robot_name
    robot_pkg_path = get_robot_package_path(task_robot_name)
    if robot_pkg_path is None:
        print(f"[ERROR] Cannot find robot package path for '{task_robot_name}'")
        return []

    task_file_path = os.path.join(robot_pkg_path, "config", "ocs2", f"{info_file_name}.info")
    print(f"[INFO] Using task file for ArmsTargetManager: {task_file_path}")

    _dual_arm, _control_base_frame, _auto_marker_frame = parse_task_info(task_file_path)
    if wbc_available:
        marker_fixed_frame = "world"
    elif _auto_marker_frame:
        marker_fixed_frame = _auto_marker_frame
    else:
        marker_fixed_frame = _control_base_frame
    print(
        f"[INFO] marker_fixed_frame={marker_fixed_frame} "
        f"(wbc={wbc_available}, control_base_frame={_control_base_frame})"
    )

    arms_target_manager = None
    arms_params = prepare_arms_target_manager_parameters(
        task_file_path=task_file_path,
        config_file_path=None,
        hand_controllers=hand_names or None,
        marker_fixed_frame=marker_fixed_frame,
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

    rviz_config_path = ocs2_common.resolve_rviz_config(ctx.robot_name, "fullbody.rviz")
    if is_arm_controller_type and marker_fixed_frame:
        try:
            with open(rviz_config_path, "r", encoding="utf-8") as handle:
                rviz_content = handle.read()
            patched_content = rviz_content.replace(
                "Fixed Frame: world", f"Fixed Frame: {marker_fixed_frame}"
            )
            tmp_name = f"full_body_{ctx.robot_name}_{ctx.robot_type or 'default'}_arm_fixedframe.rviz"
            tmp_rviz_config_path = os.path.join(tempfile.gettempdir(), tmp_name)
            with open(tmp_rviz_config_path, "w", encoding="utf-8") as handle:
                handle.write(patched_content)
            rviz_config_path = tmp_rviz_config_path
            print(
                f"[INFO] ARM type: RViz Fixed Frame={marker_fixed_frame}: {rviz_config_path}"
            )
        except Exception as exc:
            print(f"[WARN] Failed to patch RViz Fixed Frame for ARM type: {exc}")

    rviz_node = ocs2_common.build_rviz_node(
        ctx,
        rviz_config_path,
        hand_names,
        joint_controller_names,
        extra_rviz_parameters=[{"wbc_available": wbc_available}],
    )

    return ocs2_common.assemble_nodes(
        ctx,
        rviz_node=rviz_node,
        controller_stack_nodes=controller_stack_nodes,
        main_spawner=main_spawner,
        extra_spawners=hand_spawners + body_spawners + ft_spawners,
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
