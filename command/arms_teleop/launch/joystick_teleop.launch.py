#!/usr/bin/env python3

import os
import re
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _resolve_js_device(joy_dev):
    return os.path.realpath(joy_dev)


def _extract_device_id(joy_dev):
    resolved_path = _resolve_js_device(joy_dev)
    match = re.search(r'js(\d+)$', resolved_path)
    if match:
        return int(match.group(1))
    return 0


def _read_joystick_name(joy_dev):
    resolved_path = _resolve_js_device(joy_dev)
    js_name = os.path.basename(resolved_path)
    sysfs_name_path = os.path.join('/sys/class/input', js_name, 'device', 'name')

    try:
        with open(sysfs_name_path, 'r', encoding='utf-8') as f:
            return f.read().strip()
    except OSError:
        return ''


def _find_joystick_aliases(joy_dev):
    resolved_path = _resolve_js_device(joy_dev)
    by_id_dir = '/dev/input/by-id'
    aliases = []

    if not os.path.isdir(by_id_dir):
        return aliases

    for entry in sorted(os.listdir(by_id_dir)):
        full_path = os.path.join(by_id_dir, entry)
        try:
            if os.path.realpath(full_path) == resolved_path:
                aliases.append(entry)
        except OSError:
            continue

    return aliases


def _detect_config_name(config_value, config_dir, joy_dev):
    requested = (config_value or '').strip()

    if requested and requested.lower() != 'auto':
        requested_path = os.path.join(config_dir, f'{requested}.yaml')
        if os.path.exists(requested_path):
            print(f"[INFO] Using explicit joystick config: {requested}")
            return requested
        print(f"[WARN] Joystick config '{requested}' not found: {requested_path}. Falling back to auto-detection.")

    available_configs = sorted(
        os.path.splitext(name)[0]
        for name in os.listdir(config_dir)
        if name.endswith('.yaml')
    )

    joystick_name = _read_joystick_name(joy_dev)
    match_sources = []
    if joystick_name:
        match_sources.append(joystick_name)
    match_sources.extend(_find_joystick_aliases(joy_dev))

    for source in match_sources:
        lowered = source.lower()
        for config_name in sorted(
            (name for name in available_configs if name != 'default'),
            key=len,
            reverse=True,
        ):
            if config_name.lower() in lowered:
                print(f"[INFO] Auto-selected joystick config '{config_name}' for device '{source}'")
                return config_name

    if joystick_name:
        print(f"[INFO] No joystick-specific config matched '{joystick_name}', using default config")
    else:
        print(f"[INFO] Could not determine joystick name from '{joy_dev}', using default config")
    return 'default'


def launch_setup(context, *args, **kwargs):
    joy_dev = LaunchConfiguration('joy_dev').perform(context)
    config_value = LaunchConfiguration('config').perform(context)
    config_dir = os.path.join(get_package_share_directory('arms_teleop'), 'config')

    selected_config = _detect_config_name(config_value, config_dir, joy_dev)
    config_file = os.path.join(config_dir, f'{selected_config}.yaml')
    device_id = _extract_device_id(joy_dev)
    resolved_joy_dev = _resolve_js_device(joy_dev)

    print(f"[INFO] Using joystick device: {joy_dev} -> {resolved_joy_dev} (device_id={device_id})")
    print(f"[INFO] Loading joystick parameter file: {config_file}")

    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen',
        parameters=[{
            'device_id': device_id,
            'deadzone': 0.05,
            'autorepeat_rate': 20.0
        }]
    )

    joystick_teleop_node = Node(
        package='arms_teleop',
        executable='joystick_teleop',
        name='joystick_teleop',
        output='screen',
        parameters=[config_file]
    )

    return [joy_node, joystick_teleop_node]


def generate_launch_description():
    # Launch arguments
    joy_dev_arg = DeclareLaunchArgument(
        'joy_dev',
        default_value='/dev/input/js0',
        description='Joystick device file'
    )

    # Config file name argument (without .yaml extension)
    config_arg = DeclareLaunchArgument(
        'config',
        default_value='auto',
        description='Joystick config name. Use auto to select by device name, or set a file name without .yaml.'
    )

    return LaunchDescription([
        joy_dev_arg,
        config_arg,
        OpaqueFunction(function=launch_setup),
    ])
