#!/usr/bin/env python3

import os
import platform
import re
import subprocess
from ament_index_python.packages import get_package_prefix, get_package_share_directory
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
    if str(joy_dev).isdigit():
        return int(joy_dev)
    return 0


def _enumerate_joy_devices():
    try:
        joy_prefix = get_package_prefix('joy')
    except Exception:
        return {}

    executable = os.path.join(joy_prefix, 'lib', 'joy', 'joy_enumerate_devices')
    if not os.path.exists(executable):
        return {}

    try:
        result = subprocess.run(
            [executable],
            check=False,
            capture_output=True,
            text=True,
            timeout=3.0,
        )
    except (OSError, subprocess.SubprocessError):
        return {}

    devices = {}
    for line in result.stdout.splitlines():
        match = re.match(
            r'\s*(\d+)\s*:\s*[^:]+:\s*(?:true|false)\s*:\s*(?:true|false)\s*:\s*(.+?)\s*$',
            line,
            re.IGNORECASE,
        )
        if match:
            devices[int(match.group(1))] = match.group(2).strip()

    return devices


def _read_joystick_name_from_joy_enumerate(joy_dev):
    device_id = _extract_device_id(joy_dev)
    devices = _enumerate_joy_devices()
    return devices.get(device_id, '')


def _list_js_devices():
    input_dir = '/dev/input'
    if not os.path.isdir(input_dir):
        return []

    return sorted(
        name for name in os.listdir(input_dir)
        if re.fullmatch(r'js\d+', name)
    )


def _read_input_devices_from_proc():
    proc_input_path = '/proc/bus/input/devices'

    try:
        with open(proc_input_path, 'r', encoding='utf-8') as f:
            blocks = f.read().split('\n\n')
    except OSError:
        return []

    devices = []
    for block in blocks:
        name = ''
        handlers = ''

        for line in block.splitlines():
            if line.startswith('N: Name='):
                name = line.split('=', 1)[1].strip().strip('"')
            elif line.startswith('H: Handlers='):
                handlers = line.split('=', 1)[1].strip()

        if name:
            devices.append({'name': name, 'handlers': handlers})

    return devices


def _read_joystick_name_from_proc(joy_dev):
    resolved_path = _resolve_js_device(joy_dev)
    js_name = os.path.basename(resolved_path)

    for device in _read_input_devices_from_proc():
        name = device['name']
        handlers = device['handlers']
        if name and re.search(rf'(^|\s){re.escape(js_name)}($|\s)', handlers):
            return name

    return ''


def _read_joystick_name(joy_dev):
    resolved_path = _resolve_js_device(joy_dev)
    js_name = os.path.basename(resolved_path)
    sysfs_name_path = os.path.join('/sys/class/input', js_name, 'device', 'name')

    try:
        with open(sysfs_name_path, 'r', encoding='utf-8') as f:
            return f.read().strip()
    except OSError:
        return _read_joystick_name_from_proc(joy_dev)


def _find_joystick_aliases(joy_dev, allow_event_fallback=True):
    resolved_path = _resolve_js_device(joy_dev)
    by_id_dir = '/dev/input/by-id'
    aliases = []
    joystick_event_aliases = []

    if not os.path.isdir(by_id_dir):
        return aliases

    for entry in sorted(os.listdir(by_id_dir)):
        full_path = os.path.join(by_id_dir, entry)
        try:
            if os.path.realpath(full_path) == resolved_path:
                aliases.append(entry)
            elif 'joystick' in entry.lower():
                joystick_event_aliases.append(entry)
        except OSError:
            continue

    # Some containers expose /dev/input/js* but only provide by-id aliases for
    # the event joystick node. Use that as a fallback for device-name matching.
    if (
        allow_event_fallback
        and not aliases
        and len(joystick_event_aliases) == 1
        and len(_list_js_devices()) <= 1
    ):
        aliases.extend(joystick_event_aliases)

    return aliases


def _normalize_device_name(name):
    return re.sub(r'[^a-z0-9]+', ' ', name.lower()).strip()


def _is_arm_architecture():
    machine = platform.machine().lower()
    return machine in ('aarch64', 'arm64') or machine.startswith('arm')


def _ps4_controller_config_name():
    return 'gamesir_two_arm' if _is_arm_architecture() else 'gamesir_two'


def _find_known_joystick_names_from_proc(known_device_names):
    matches = []

    for device in _read_input_devices_from_proc():
        name = device['name']
        handlers = device['handlers'].split()
        normalized = _normalize_device_name(name)
        lowered = name.lower()

        # Ignore keyboard companion interfaces such as "System Control" and
        # "Consumer Control"; they are not the joystick stream joy_node opens.
        if 'kbd' in handlers:
            continue

        for device_name in known_device_names:
            if device_name in lowered or _normalize_device_name(device_name) in normalized:
                matches.append(name)
                break

    return sorted(set(matches))


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

    device_config_aliases = {
        'gamesir-dongle': 'gamesir',
        'gamesir': 'gamesir',
        'logitech cordless rumblepad 2': 'default',
        'logitech logitech cordless rumblepad 2': 'default',
        'logitech f710 gamepad': 'default',
        'logitech': 'default',
        'ps4 controller': _ps4_controller_config_name(),
        'sony interactive entertainment wireless controller': 'gamesir_two',
    }

    joystick_name = _read_joystick_name(joy_dev)
    enumerated_name = _read_joystick_name_from_joy_enumerate(joy_dev)
    proc_known_names = []
    match_sources = []
    allow_event_alias_fallback = True

    if enumerated_name:
        match_sources.append(enumerated_name)
    if joystick_name and joystick_name not in match_sources:
        match_sources.append(joystick_name)
    if not match_sources:
        proc_known_names = _find_known_joystick_names_from_proc(device_config_aliases.keys())
        if len(proc_known_names) == 1:
            match_sources.append(proc_known_names[0])
        elif len(proc_known_names) > 1:
            allow_event_alias_fallback = False
            print(
                "[INFO] Found multiple known joystick candidates "
                f"{proc_known_names}; cannot infer which one is '{joy_dev}' from /proc alone."
            )

    match_sources.extend(_find_joystick_aliases(joy_dev, allow_event_alias_fallback))

    for source in match_sources:
        lowered = source.lower()
        normalized = _normalize_device_name(source)
        for device_name, config_name in device_config_aliases.items():
            config_path = os.path.join(config_dir, f'{config_name}.yaml')
            if (
                (device_name in lowered or _normalize_device_name(device_name) in normalized)
                and os.path.exists(config_path)
            ):
                print(f"[INFO] Auto-selected joystick config '{config_name}' for device '{source}'")
                return config_name

        for config_name in sorted(
            (name for name in available_configs if name != 'default'),
            key=len,
            reverse=True,
        ):
            if config_name.lower() in lowered or _normalize_device_name(config_name) in normalized:
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
