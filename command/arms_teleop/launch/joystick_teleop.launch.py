#!/usr/bin/env python3

import os
import re
import subprocess

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

JOY_MAPPING_DIR = 'joy_mapping'
ENUMERATE_LINE_RE = re.compile(
    r'^\s*(\d+)\s*:\s*([^:]+)\s*:\s*(true|false)\s*:\s*(true|false)\s*:\s*(.+?)\s*$',
    re.IGNORECASE,
)


def _parse_bool_text(value: str):
    text = (value or '').strip().lower()
    if text in ('true', '1', 'yes', 'on'):
        return True
    if text in ('false', '0', 'no', 'off'):
        return False
    return None


def _extract_device_id(joy_dev: str) -> int:
    resolved_path = os.path.realpath(joy_dev)
    match = re.search(r'js(\d+)$', resolved_path)
    if match:
        return int(match.group(1))
    if str(joy_dev).isdigit():
        return int(joy_dev)
    return 0


def _enumerate_devices():
    executable = os.path.join(get_package_prefix('joy'), 'lib', 'joy', 'joy_enumerate_devices')
    if not os.path.exists(executable):
        return []

    try:
        result = subprocess.run(
            [executable],
            check=False,
            capture_output=True,
            text=True,
            timeout=3.0,
        )
    except (OSError, subprocess.SubprocessError):
        return []

    devices = []
    for line in result.stdout.splitlines():
        match = ENUMERATE_LINE_RE.match(line)
        if not match:
            continue
        devices.append({
            'device_id': int(match.group(1)),
            'guid': match.group(2).strip(),
            'gamepad': _parse_bool_text(match.group(3)),
            'mapped': _parse_bool_text(match.group(4)),
            'name': match.group(5).strip(),
        })
    return devices


def _find_device_by_joy_dev(joy_dev: str):
    device_id = _extract_device_id(joy_dev)
    for device in _enumerate_devices():
        if device['device_id'] == device_id:
            return device
    return None


def _resolve_joy_driver(mapped, joy_driver_arg: str) -> str:
    requested = (joy_driver_arg or 'auto').strip().lower()
    if requested in ('game_controller', 'game_controller_node', 'gc'):
        return 'game_controller'
    if requested in ('joy', 'joy_node'):
        return 'joy'
    if mapped is True:
        return 'game_controller'
    return 'joy'


def launch_setup(context, *args, **kwargs):
    joy_dev = LaunchConfiguration('joy_dev').perform(context)
    config_value = LaunchConfiguration('config').perform(context)
    joy_driver_arg = LaunchConfiguration('joy_driver').perform(context)

    package_config_dir = os.path.join(get_package_share_directory('arms_teleop'), 'config')
    mapping_dir = os.path.join(package_config_dir, JOY_MAPPING_DIR)

    device = _find_device_by_joy_dev(joy_dev) or {
        'device_id': _extract_device_id(joy_dev),
        'name': '',
        'mapped': None,
        'gamepad': None,
    }
    joy_driver = _resolve_joy_driver(device.get('mapped'), joy_driver_arg)
    config_is_auto = (config_value or 'auto').strip().lower() == 'auto'

    if not config_is_auto:
        config_path = os.path.join(mapping_dir, f'{config_value.strip()}.yaml')
        if os.path.exists(config_path):
            selected_config = config_value.strip()
            print(f'[INFO] Using explicit joystick config: {selected_config}')
        else:
            print(f"[WARN] Joystick config '{config_value}' not found under {mapping_dir}.")
            selected_config = 'generic_gamepad' if joy_driver == 'game_controller' else 'default'
    elif joy_driver == 'game_controller':
        selected_config = 'generic_gamepad'
        print('[INFO] Using game_controller_node with generic_gamepad (SDL standard layout)')
    else:
        selected_config = 'default'
        print('[INFO] Using joy_node with default mapping')

    config_file = os.path.join(mapping_dir, f'{selected_config}.yaml')
    device_id = _extract_device_id(joy_dev)
    resolved_joy_dev = os.path.realpath(joy_dev)

    joy_executable = 'game_controller_node' if joy_driver == 'game_controller' else 'joy_node'
    print(
        f'[INFO] Joy driver: {joy_executable} '
        f'(device_id={device_id}, joy_dev={joy_dev} -> {resolved_joy_dev})'
    )
    print(f'[INFO] Loading joystick parameter file: {config_file}')
    if device.get('name'):
        print(
            f"[INFO] Joystick device: '{device['name']}' "
            f"(mapped={device.get('mapped')}, gamepad={device.get('gamepad')})"
        )

    joy_input_node = Node(
        package='joy',
        executable=joy_executable,
        name='joy_node',
        output='screen',
        parameters=[{
            'device_id': device_id,
            'deadzone': 0.05,
            'autorepeat_rate': 20.0,
        }],
    )

    joystick_teleop_node = Node(
        package='arms_teleop',
        executable='joystick_teleop',
        name='joystick_teleop',
        output='screen',
        parameters=[config_file],
    )

    return [joy_input_node, joystick_teleop_node]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'joy_dev',
            default_value='/dev/input/js0',
            description='Joystick device file (/dev/input/jsN)',
        ),
        DeclareLaunchArgument(
            'joy_driver',
            default_value='auto',
            description='Joy input driver: auto (game_controller when Mapped), game_controller, or joy',
        ),
        DeclareLaunchArgument(
            'config',
            default_value='auto',
            description='Mapping YAML stem under config/joy_mapping/, or auto',
        ),
        OpaqueFunction(function=launch_setup),
    ])
