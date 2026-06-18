"""Shared setup for ocs2_arm_controller launch files."""

from __future__ import annotations

import os
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Tuple

from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

from robot_common_launch import (
    RobotConfigMeta,
    build_planning_urdf_launch_params,
    create_controller_manager_nodes,
    create_controller_spawners,
    detect_and_spawn_controllers,
    detect_controllers,
    forward_robot_launch_args,
    get_robot_package_path,
    get_ros2_control_robot_description,
    load_robot_config,
    load_robot_profile,
    parse_launch_mode,
    prepare_ros2_controllers_override_path,
    resolve_control_patch,
    resolve_control_sides,
    resolve_profile_path,
    resolve_robot_variant,
    write_spawner_controller_param_file,
)


@dataclass
class Ocs2ControlContext:
    robot_name: str
    robot_type: str
    hardware: str
    world: str
    use_sim_time: bool
    profile_path: str
    control_left: str
    control_right: str
    control_patch: dict
    config: dict
    meta: RobotConfigMeta
    launch_mode: str
    rviz_only: bool
    use_rviz: bool
    launch_configurations: dict = field(default_factory=dict)


def build_ocs2_control_context(context) -> Ocs2ControlContext:
    """Load profile, ros2_control config, and launch mode from launch context."""
    configs = context.launch_configurations
    robot_name = configs["robot"]
    robot_type = configs.get("type", "")
    hardware = configs.get("hardware", "mock_components")
    world = configs.get("world", "dart")
    use_sim_time = hardware in ("gz", "isaac")

    launch_mode, rviz_only, use_rviz = parse_launch_mode(context)

    profile_path = resolve_profile_path(configs)
    profile = load_robot_profile(profile_path) if profile_path else {}
    control_left, control_right = resolve_control_sides(configs, profile)
    control_patch = resolve_control_patch(profile)
    robot_variant = resolve_robot_variant(configs, profile)

    config, _path, meta = load_robot_config(
        robot_name,
        "ros2_control",
        robot_type,
        control_left=control_left,
        control_right=control_right,
        control_patch=control_patch,
        robot_variant=robot_variant,
    )

    return Ocs2ControlContext(
        robot_name=robot_name,
        robot_type=robot_type,
        hardware=hardware,
        world=world,
        use_sim_time=use_sim_time,
        profile_path=profile_path or "",
        control_left=control_left,
        control_right=control_right,
        control_patch=control_patch,
        config=config or {},
        meta=meta,
        launch_mode=launch_mode,
        rviz_only=rviz_only,
        use_rviz=use_rviz,
        launch_configurations=dict(configs),
    )


def resolve_planning_robot_name_from_config(
    config: dict,
    controller_key: str,
    robot_name: str,
) -> str:
    planning_robot_name = robot_name
    if not config:
        return planning_robot_name
    try:
        params = config.get(controller_key, {}).get("ros__parameters", {})
        config_robot_name = params.get("robot_name")
        if config_robot_name:
            planning_robot_name = config_robot_name
            if config_robot_name != robot_name:
                print(
                    f"[INFO] Using robot_name from config for planning URDF: "
                    f"{planning_robot_name} (launch arg: {robot_name})"
                )
    except KeyError:
        pass
    return planning_robot_name


def validate_planning_urdf(
    planning_robot_name: str,
    launch_configurations: dict,
    hardware: str,
    profile_path: Optional[str],
    *,
    planning_scope: str,
) -> Optional[dict]:
    planning_urdf_params = build_planning_urdf_launch_params(
        planning_robot_name,
        launch_configurations,
        hardware,
        profile_path or None,
        planning_scope=planning_scope,
    )
    plan_path = (planning_urdf_params.get("planning_urdf_path") or "").strip()
    if (
        planning_urdf_params.get("planning_urdf_variant") != "xacro"
        or not plan_path
        or not os.path.isfile(plan_path)
    ):
        print(
            f"[ERROR] OCS2 requires xacro planning URDF for '{planning_robot_name}' "
            f"(scope={planning_scope}). variant="
            f"{planning_urdf_params.get('planning_urdf_variant')!r} path={plan_path!r}"
        )
        return None
    return planning_urdf_params


def create_controller_stack_nodes(
    ctx: Ocs2ControlContext,
    context,
    *,
    ocs2_planning_param_file: str,
) -> List[Any]:
    """robot_state_publisher + ros2_control_node + joint_state_broadcaster."""
    use_include = (
        ctx.launch_configurations.get("use_controller_manager_include", "false").lower()
        in ("true", "1", "yes")
    )
    override_path = prepare_ros2_controllers_override_path(ctx.meta)
    if override_path:
        print(f"[INFO] Preloaded ros2_control config: {override_path}")

    if use_include:
        forward_args = forward_robot_launch_args(context)
        return [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [
                        os.path.join(
                            get_package_share_directory("robot_common_launch"),
                            "launch",
                        ),
                        "/controller_manager.launch.py",
                    ]
                ),
                launch_arguments=forward_args
                + [
                    ("use_sim_time", str(ctx.use_sim_time)),
                    ("world", ctx.world),
                    ("ocs2_planning_param_file", ocs2_planning_param_file),
                    ("ros2_controllers_override", override_path),
                ],
            )
        ]

    return create_controller_manager_nodes(
        robot_name=ctx.robot_name,
        robot_type=ctx.robot_type,
        hardware=ctx.hardware,
        use_sim_time=ctx.use_sim_time,
        world=ctx.world,
        ocs2_planning_param_file=ocs2_planning_param_file,
        ros2_controllers_override=override_path,
        launch_configurations=ctx.launch_configurations,
        preloaded_config=ctx.config,
        preloaded_meta=ctx.meta,
    )


def setup_hand_controllers(
    ctx: Ocs2ControlContext,
    enable_gripper: bool,
) -> Tuple[List[dict], List[Node]]:
    if not enable_gripper:
        return [], []

    robot_description = get_ros2_control_robot_description(
        ctx.robot_name,
        robot_type=ctx.robot_type,
        hardware=ctx.hardware,
        launch_configurations=ctx.launch_configurations,
        robot_profile=ctx.profile_path or None,
    )
    return detect_and_spawn_controllers(
        ctx.config,
        ["hand", "gripper"],
        robot_description=robot_description,
        use_sim_time=ctx.use_sim_time,
    )


def setup_body_controllers(
    ctx: Ocs2ControlContext,
    patterns: List[str],
) -> Tuple[List[dict], List[Node]]:
    controllers = detect_controllers(
        ctx.robot_name,
        ctx.robot_type,
        patterns,
        control_left=ctx.control_left,
        control_right=ctx.control_right,
        control_patch=ctx.control_patch,
        ros2_control_config=ctx.config,
    )
    return controllers, create_controller_spawners(controllers, ctx.use_sim_time)


def resolve_rviz_config(
    robot_name: str,
    rviz_filename: str,
    fallback_filename: str = "demo.rviz",
) -> str:
    robot_pkg_path = get_robot_package_path(robot_name)
    if robot_pkg_path is not None:
        robot_rviz_config = os.path.join(robot_pkg_path, "config", "rviz", rviz_filename)
        print(f"[INFO] Checking for rviz config in robot description package: {robot_rviz_config}")
        if os.path.exists(robot_rviz_config):
            print(f"[INFO] Using rviz config from robot description package: {robot_rviz_config}")
            return robot_rviz_config

    rviz_base = os.path.join(get_package_share_directory("ocs2_arm_controller"), "config")
    rviz_config_path = os.path.join(rviz_base, fallback_filename)
    print(f"[INFO] Using default rviz config: {rviz_config_path}")
    return rviz_config_path


def build_rviz_node(
    ctx: Ocs2ControlContext,
    rviz_config_path: str,
    hand_controller_names: List[str],
    joint_controller_names: List[str],
    *,
    extra_rviz_parameters: Optional[List[dict]] = None,
) -> Optional[Node]:
    if not ctx.use_rviz:
        return None

    rviz_parameters: List[dict] = [{"use_sim_time": ctx.use_sim_time}]
    if hand_controller_names:
        rviz_parameters.append({"hand_controllers": hand_controller_names})
    rviz_parameters.append({"joint_controllers": joint_controller_names})
    if extra_rviz_parameters:
        rviz_parameters.extend(extra_rviz_parameters)

    return Node(
        package="rviz2",
        executable="rviz2",
        output="log",
        arguments=["-d", rviz_config_path],
        parameters=rviz_parameters,
    )


def create_main_controller_spawner(
    controller_name: str,
    planning_urdf_params: dict,
) -> Tuple[Node, str]:
    param_file = write_spawner_controller_param_file(
        controller_name, planning_urdf_params, quiet=True
    )
    plan_path = (planning_urdf_params.get("planning_urdf_path") or "").strip()
    print(f"[INFO] OCS2 planning URDF: {plan_path} (params: {param_file})")
    spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[controller_name, "-p", param_file],
        output="screen",
    )
    return spawner, param_file


def assemble_nodes(
    ctx: Ocs2ControlContext,
    *,
    rviz_node: Optional[Node],
    controller_stack_nodes: List[Any],
    main_spawner: Optional[Node],
    extra_spawners: List[Node],
    optional_nodes: Optional[List[Any]] = None,
) -> List[Any]:
    nodes: List[Any] = []
    optional_nodes = optional_nodes or []

    if ctx.rviz_only:
        if rviz_node:
            nodes.append(rviz_node)
        return nodes

    if rviz_node:
        nodes.append(rviz_node)
    nodes.extend(controller_stack_nodes)
    if main_spawner:
        nodes.append(main_spawner)
    nodes.extend(optional_nodes)
    nodes.extend(extra_spawners)
    return nodes
