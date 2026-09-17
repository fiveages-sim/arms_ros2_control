#!/usr/bin/env python3
"""Static payload identification from raw WrenchStamped and timestamped TF.

ROS is imported only for online acquisition; fit and plan validation work offline.
Supports standard FollowJointTrajectory, repository MOVEJ, and manual sampling.
"""
import argparse
import csv
import json
import select
import signal
from collections import deque
from datetime import datetime, timezone
from pathlib import Path
import sys
import time

import numpy as np
import yaml

COLUMNS = ['pose_id', 'stamp_sec', 'sensor_frame', 'gravity_frame',
           'gx', 'gy', 'gz', 'fx', 'fy', 'fz', 'tx', 'ty', 'tz']
DEFAULTS = {
    'joint_state_topic': '/joint_states',
    'motion_backend': 'follow_joint_trajectory',
    'trajectory_action': '/joint_trajectory_controller/follow_joint_trajectory',
    'fsm_command_topic': '/fsm_command', 'fsm_state_topic': '/fsm_state',
    'gravity_vector': [0.0, 0.0, -9.81],
    'move_seconds': 5.0, 'max_velocity': 0.15,
    'max_acceleration': 0.3, 'max_jerk': 1.0, 'max_step_rad': 0.7,
    'position_tolerance': 0.02, 'still_velocity': 0.01,
    'settle_seconds': 1.5, 'sample_seconds': 2.0, 'min_samples': 30,
    'data_timeout': 0.5, 'wait_timeout': 30.0,
    'max_force_std': 0.5, 'max_torque_std': 0.05,
    'max_force_rms': 0.5, 'max_torque_rms': 0.05,
    'max_condition': 1000.0,
}


def vector(value, length, name):
    a = np.asarray(value, dtype=float)
    if a.shape != (length,) or not np.all(np.isfinite(a)):
        raise ValueError(f'{name}: expected {length} finite numbers')
    return a


def skew(v):
    x, y, z = v
    return np.array([[0., -z, y], [z, 0., -x], [-y, x, 0.]])


def rotation_xyzw(q):
    q = vector(q, 4, 'quaternion')
    norm = np.linalg.norm(q)
    if norm < 1e-9:
        raise ValueError('Invalid zero quaternion')
    x, y, z, w = q / norm
    return np.array([
        [1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
        [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)],
    ])


def solve_scaled(a, b, limit):
    scale = np.linalg.norm(a, axis=0)
    if np.any(scale < 1e-12):
        raise ValueError('Insufficient orientation diversity: unobservable parameter')
    normalized = a / scale
    x, _, rank, singular = np.linalg.lstsq(normalized, b, rcond=None)
    condition = float(singular[0] / singular[-1]) if singular[-1] > 0 else float('inf')
    if rank != a.shape[1] or condition > limit:
        raise ValueError('Insufficient orientation diversity: '
                         f'rank={rank}/{a.shape[1]}, condition={condition:.3g}. '
                         'Tilt the sensor about two different axes; yaw alone is insufficient.')
    return x / scale, condition


def estimate_payload(gravity, wrench, max_condition=1000.0):
    """Equal weight per static pose. Biases are in the unmodified raw convention.

    F = signed_mass * g + b_f; T = -skew(g) * signed_first_moment + b_t.
    A common force/torque sign is inferred from signed_mass; CoM = h / mass.
    """
    g = np.asarray(gravity, dtype=float)
    w = np.asarray(wrench, dtype=float)
    if g.ndim != 2 or g.shape[1] != 3 or len(g) < 6 or w.shape != (len(g), 6):
        raise ValueError('Need at least six poses, gravity Nx3 and wrench Nx6')
    if not np.all(np.isfinite(g)) or not np.all(np.isfinite(w)):
        raise ValueError('Non-finite measurement')
    if not np.isfinite(max_condition) or max_condition <= 1:
        raise ValueError('max_condition must be finite and > 1')
    a_f = np.vstack([np.column_stack((v, np.eye(3))) for v in g])
    a_t = np.vstack([np.column_stack((-skew(v), np.eye(3))) for v in g])
    f, cond_f = solve_scaled(a_f, w[:, :3].reshape(-1), max_condition)
    t, cond_t = solve_scaled(a_t, w[:, 3:].reshape(-1), max_condition)
    signed_mass = float(f[0])
    if abs(signed_mass) < 0.001:
        raise ValueError('Estimated mass below 1 g; CoM is not reliable')
    com = t[:3] / signed_mass
    residual = np.column_stack(((a_f @ f).reshape(-1, 3),
                                (a_t @ t).reshape(-1, 3))) - w
    force_rms = float(np.sqrt(np.mean(np.sum(residual[:, :3]**2, axis=1))))
    torque_rms = float(np.sqrt(np.mean(np.sum(residual[:, 3:]**2, axis=1))))
    return {
        'mass_kg': abs(signed_mass), 'wrench_sign': 1 if signed_mass > 0 else -1,
        'center_of_mass_m': com.tolist(), 'center_of_mass_mm': (com * 1000).tolist(),
        'force_bias_N': f[1:].tolist(), 'torque_bias_Nm': t[3:].tolist(),
        'force_residual_rms_N': force_rms, 'torque_residual_rms_Nm': torque_rms,
        'residuals_per_pose': residual.tolist(),
        'force_condition_scaled': cond_f, 'torque_condition_scaled': cond_t,
        'pose_count': len(g),
    }


def fit_rows(rows, max_condition=1000.0, max_force_rms=0.5, max_torque_rms=0.05):
    if not rows:
        raise ValueError('No samples')
    frames = {(r['sensor_frame'], r['gravity_frame']) for r in rows}
    if len(frames) != 1 or any(not part for part in next(iter(frames))):
        raise ValueError('All samples must use the same nonempty sensor/gravity frames')
    groups = {}
    for row in rows:
        values = vector([row[k] for k in COLUMNS[4:]], 9, 'CSV sample')
        groups.setdefault(str(row['pose_id']), []).append(values)
    means = np.array([np.mean(samples, axis=0) for samples in groups.values()])
    result = estimate_payload(means[:, :3], means[:, 3:], max_condition)
    result.update(sensor_frame=rows[0]['sensor_frame'], gravity_frame=rows[0]['gravity_frame'],
                  created_utc=datetime.now(timezone.utc).isoformat(),
                  sample_count=len(rows), pose_ids=list(groups),
                  samples_per_pose=[len(a) for a in groups.values()])
    result['valid'] = (result['force_residual_rms_N'] <= max_force_rms
                       and result['torque_residual_rms_Nm'] <= max_torque_rms)
    result['residual_limits'] = {'force_N': max_force_rms, 'torque_Nm': max_torque_rms}
    result['model'] = 'F = sign*m*g + bias_F; T = sign*(m*com) cross g + bias_T'
    return result


def load_plan(path):
    with open(path, encoding='utf-8') as f:
        raw = yaml.safe_load(f)
    if not isinstance(raw, dict):
        raise ValueError('Plan must be a YAML mapping')
    allowed = set(DEFAULTS) | {'joint_names', 'poses', 'wrench_topic', 'sensor_frame',
                               'gravity_frame', 'schema_version'}
    unknown = set(raw) - allowed
    if unknown:
        raise ValueError(f'Unknown plan keys: {sorted(unknown)}')
    cfg = {**DEFAULTS, **raw}
    # Plans written by the initial controller-local version used only MOVEJ.
    if raw.get('schema_version') == 1 and 'motion_backend' not in raw:
        cfg['motion_backend'] = 'arms'
        cfg['trajectory_action'] = raw.get('trajectory_action', '/joint_trajectory_with_para')
    if cfg['motion_backend'] not in ('arms', 'follow_joint_trajectory'):
        raise ValueError('Automatic plans require arms or follow_joint_trajectory backend')
    names = cfg.get('joint_names', [])
    if (not isinstance(names, list) or not names
            or any(not isinstance(n, str) or not n for n in names)
            or len(set(names)) != len(names)):
        raise ValueError('joint_names must be nonempty, unique strings')
    for name in ('wrench_topic', 'sensor_frame', 'gravity_frame', 'joint_state_topic',
                 'trajectory_action', 'fsm_command_topic', 'fsm_state_topic'):
        if not isinstance(cfg.get(name), str) or not cfg[name].strip():
            raise ValueError(f'Missing/invalid {name}')
    if 'filtered' in cfg['wrench_topic'].lower():
        raise ValueError('Use the raw wrench topic, not wrench_filtered')
    for name in DEFAULTS:
        if name in ('gravity_vector',) or isinstance(DEFAULTS[name], str):
            continue
        val = cfg[name]
        if isinstance(val, bool) or not isinstance(val, (int, float)) or not np.isfinite(val) or val <= 0:
            raise ValueError(f'{name} must be positive and finite')
    if isinstance(cfg['min_samples'], float) or cfg['min_samples'] < 3:
        raise ValueError('min_samples must be an integer >= 3')
    if cfg['max_condition'] <= 1:
        raise ValueError('max_condition must be > 1')
    cfg['gravity_vector'] = vector(cfg['gravity_vector'], 3, 'gravity_vector').tolist()
    if np.linalg.norm(cfg['gravity_vector']) < 1e-3:
        raise ValueError('gravity_vector cannot be zero')
    poses = cfg.get('poses')
    if not isinstance(poses, list) or len(poses) < 6:
        raise ValueError('At least six pose vectors are required')
    cfg['poses'] = [vector(q, len(names), 'pose').tolist() for q in poses]
    validate_steps(cfg['poses'][0], cfg)
    return cfg


def validate_steps(current, cfg):
    path = np.vstack([current, *cfg['poses']])
    steps = np.max(np.abs(np.diff(path, axis=0)), axis=1)
    if np.any(steps > cfg['max_step_rad']):
        index = int(np.argmax(steps))
        raise ValueError(f'Move to pose {index} exceeds max_step_rad: {steps[index]:.3f} rad')


def generate_poses(current, names, sweep, angle):
    if len(sweep) != 2 or sweep[0] == sweep[1] or not set(sweep).issubset(names):
        raise ValueError('Choose two distinct sweep joints from joint_names')
    if not np.isfinite(angle) or angle <= 0:
        raise ValueError('angle-deg must be positive and finite')
    indices = [names.index(n) for n in sweep]
    # Traversal around a small grid; no assumed robot geometry or joint naming.
    offsets = [(0, 0), (1, 0), (1, 1), (0, 1), (-1, 1),
               (-1, 0), (-1, -1), (0, -1), (1, -1)]
    poses = []
    for x, y in offsets:
        q = np.array(current, dtype=float)
        q[indices] += np.deg2rad(angle) * np.array([x, y])
        poses.append(q.tolist())
    return poses


def make_ros_node(cfg, enable_motion=True):
    import rclpy
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.time import Time
    from geometry_msgs.msg import WrenchStamped
    from sensor_msgs.msg import JointState
    from tf2_ros import Buffer, TransformListener, TransformException

    class Sampler(Node):
        def __init__(self):
            super().__init__('identify_payload', parameter_overrides=[
                Parameter('use_sim_time', value=cfg.get('use_sim_time', False))])
            self.q = None
            self.joint_stamp = None
            self.joint_received = 0.
            self.speed = float('inf')
            self.wrenches = deque(maxlen=2000)
            self.last_wrench_stamp = -1
            self.last_wrench_received = 0.
            self.error = None
            self.compliance_received = 0.
            self.compliance_status = None
            if cfg.get('compliance_status_topic'):
                from arms_ros2_control_msgs.msg import ComplianceForceStatus
                self.create_subscription(ComplianceForceStatus, cfg['compliance_status_topic'],
                                         self.on_compliance_status, 10)
            self.buffer = Buffer()
            self.listener = TransformListener(self.buffer, self)
            self.create_subscription(JointState, cfg['joint_state_topic'], self.on_joint,
                                     qos_profile_sensor_data)
            self.create_subscription(WrenchStamped, cfg['wrench_topic'], self.on_wrench,
                                     qos_profile_sensor_data)
            from payload_motion import create_motion
            self.motion = create_motion(self, cfg) if enable_motion else None

        def on_compliance_status(self, msg):
            self.compliance_received = time.monotonic()
            self.compliance_status = msg

        def guard_compliance(self):
            if not cfg.get('compliance_status_topic'):
                return
            msg = self.compliance_status
            if msg is None or time.monotonic() - self.compliance_received > cfg['data_timeout']:
                raise RuntimeError('COMPLIANCE status unavailable or stale; collection stopped')
            if msg.identification_active:
                raise RuntimeError('Another identification is active; collection stopped')
            if any(not np.isfinite(v) or v != 0. for v in msg.task_selection):
                raise RuntimeError('Payload identification requires all six axes in position control (S=0)')

        def on_joint(self, msg):
            # Some systems publish partial joint states: wait for a message with all chosen joints.
            if not set(cfg['joint_names']).issubset(msg.name):
                return
            try:
                indices = [msg.name.index(n) for n in cfg['joint_names']]
                q = vector([msg.position[i] for i in indices], len(indices), 'joint positions')
                stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
                age = self.get_clock().now().nanoseconds * 1e-9 - stamp
                if stamp <= 0 or age < -0.05 or age > cfg['data_timeout']:
                    return
                if self.joint_stamp is not None:
                    dt = stamp - self.joint_stamp
                    if dt <= 0:
                        return
                    self.speed = float(np.max(np.abs(q - self.q)) / dt)
                    if len(msg.velocity) == len(msg.name):
                        velocity = vector([msg.velocity[i] for i in indices], len(indices), 'joint velocity')
                        self.speed = max(self.speed, float(np.max(np.abs(velocity))))
                self.q, self.joint_stamp = q, stamp
                self.joint_received = time.monotonic()
            except (ValueError, IndexError) as exc:
                self.error = f'Invalid joint state: {exc}'

        def on_wrench(self, msg):
            stamp = msg.header.stamp.sec * 1000000000 + msg.header.stamp.nanosec
            age = (self.get_clock().now().nanoseconds - stamp) * 1e-9
            if stamp <= self.last_wrench_stamp or stamp <= 0 or age > cfg['data_timeout'] or age < -0.05:
                return
            if not msg.header.frame_id or (cfg.get('sensor_frame') and msg.header.frame_id != cfg['sensor_frame']):
                self.error = f'Unexpected wrench frame: {msg.header.frame_id!r}'
                return
            self.last_wrench_stamp = stamp
            self.last_wrench_received = time.monotonic()
            self.wrenches.append(msg)

        def spin(self):
            if not rclpy.ok():
                raise RuntimeError('ROS context stopped')
            rclpy.spin_once(self, timeout_sec=0.01)
            if self.error:
                raise RuntimeError(self.error)

        def wait(self, predicate, timeout, label, check=None):
            deadline = time.monotonic() + timeout
            while not predicate():
                if time.monotonic() >= deadline:
                    raise TimeoutError(label)
                self.spin()
                if check:
                    check()

        def fresh(self):
            now = time.monotonic()
            return (self.q is not None and now - self.joint_received < cfg['data_timeout']
                    and now - self.last_wrench_received < cfg['data_timeout'])

        def guard_motion(self):
            self.guard_compliance()
            if not self.fresh():
                raise RuntimeError('Joint/wrench stream became stale')
            if self.motion is not None:
                self.motion.guard()

        def sample(self, after_ns=0):
            # Exact wrench timestamp; never substitute latest TF for missing history.
            while self.wrenches:
                msg = self.wrenches[0]
                stamp = msg.header.stamp.sec * 1000000000 + msg.header.stamp.nanosec
                if stamp < after_ns or (self.get_clock().now().nanoseconds - stamp) * 1e-9 > cfg['data_timeout']:
                    self.wrenches.popleft()
                    continue
                try:
                    transform = self.buffer.lookup_transform(
                        msg.header.frame_id, cfg['gravity_frame'], Time.from_msg(msg.header.stamp))
                except TransformException:
                    return None  # Allow later TF to interpolate this buffered sample.
                self.wrenches.popleft()
                q = transform.transform.rotation
                gravity = rotation_xyzw([q.x, q.y, q.z, q.w]) @ cfg['gravity_vector']
                f, t = msg.wrench.force, msg.wrench.torque
                wrench = vector([f.x, f.y, f.z, t.x, t.y, t.z], 6, 'raw wrench')
                return {'stamp_sec': stamp * 1e-9, 'sensor_frame': msg.header.frame_id,
                        'gravity_frame': cfg['gravity_frame'],
                        **dict(zip(COLUMNS[4:], [*gravity.tolist(), *wrench.tolist()]))}
            return None

        def ready(self):
            self.wait(lambda: self.fresh() and self.speed <= cfg['still_velocity'],
                      cfg['wait_timeout'], 'Waiting for fresh, stationary joint states and raw wrench')
            if cfg.get('compliance_status_topic'):
                self.wait(lambda: self.compliance_status is not None, cfg['wait_timeout'],
                          'Waiting for COMPLIANCE status')
                self.guard_compliance()
            if self.motion is not None:
                self.motion.ready()
            first = []
            def have_tf():
                row = self.sample()
                if row:
                    first.append(row)
                return bool(first)
            self.wait(have_tf, cfg['wait_timeout'], 'No TF at raw wrench timestamps')
            return first[0]['sensor_frame']

        def start_motion(self):
            if self.motion is not None:
                self.motion.start()

        def move(self, target):
            self.guard_motion()
            validate_steps(self.q, {**cfg, 'poses': [target]})
            duration = max(cfg['move_seconds'], 2 * float(np.max(np.abs(target-self.q))) / cfg['max_velocity'])
            self.motion.move(target, duration)

        def settled(self, target):
            self.guard_motion()
            return (float(np.max(np.abs(self.q-target))) <= cfg['position_tolerance']
                    and self.speed <= cfg['still_velocity'])

        def collect(self, target, pose_id, writer, stream):
            # Completion of the planned trajectory alone does not prove measured convergence.
            stable_since = None
            def is_settled():
                nonlocal stable_since
                if not self.settled(target):
                    stable_since = None
                elif stable_since is None:
                    stable_since = time.monotonic()
                return stable_since is not None and time.monotonic()-stable_since >= cfg['settle_seconds']
            self.wait(is_settled, cfg['wait_timeout'], 'Measured joints failed to settle')
            after_ns = self.get_clock().now().nanoseconds
            self.wrenches.clear()
            deadline = time.monotonic() + cfg['sample_seconds']
            rows = []
            while time.monotonic() < deadline:
                self.spin()
                if not self.settled(target):
                    raise RuntimeError('Arm moved during static sampling')
                row = self.sample(after_ns)
                if row is not None:
                    row['pose_id'] = pose_id
                    rows.append(row)
            if len(rows) < cfg['min_samples']:
                raise RuntimeError(f'Only {len(rows)} valid samples at pose {pose_id}; need {cfg["min_samples"]}')
            values = np.array([[r[k] for k in COLUMNS[7:]] for r in rows])
            std = np.std(values, axis=0, ddof=1)
            if np.linalg.norm(std[:3]) > cfg['max_force_std'] or np.linalg.norm(std[3:]) > cfg['max_torque_std']:
                raise RuntimeError(f'Wrench not stable at pose {pose_id}, axis std={std.tolist()}')
            writer.writerows(rows)
            stream.flush()
            return rows

        def stop(self):
            if self.motion is not None:
                self.motion.stop()

    return Sampler()


def save_result(result, output):
    with open(output, 'x', encoding='utf-8') as f:
        yaml.safe_dump(result, f, sort_keys=False, allow_unicode=True)
    print(yaml.safe_dump(result, sort_keys=False, allow_unicode=True))
    if not result['valid']:
        raise ValueError(f'Residual exceeds limits; {output} saved with valid: false')


def panel_event(event, **fields):
    print(json.dumps({'event': event, **fields}, ensure_ascii=False), flush=True)


def wait_panel_sample(node, pose, count):
    """Keep ROS subscriptions alive while RViz waits for the next sample click."""
    panel_event('ready', pose=pose, total=count)
    while True:
        node.spin()
        node.guard_compliance()
        if select.select([sys.stdin], [], [], 0)[0]:
            command = sys.stdin.readline()
            if not command or command.strip() == 'stop':
                raise KeyboardInterrupt()
            if command.strip() == 'sample':
                panel_event('sampling', pose=pose, total=count)
                return


def run_live(args):
    if args.command in ('make-plan', 'collect'):
        cfg = {**DEFAULTS, 'joint_names': args.joints, 'wrench_topic': args.wrench_topic,
               'gravity_frame': args.gravity_frame, 'gravity_vector': args.gravity_vector,
               'sensor_frame': '', 'joint_state_topic': args.joint_state_topic,
               'motion_backend': 'manual' if args.command == 'collect' else args.motion_backend}
        if args.command == 'collect' and args.panel:
            cfg['compliance_status_topic'] = args.compliance_status_topic
            cfg['use_sim_time'] = args.use_sim_time
        if args.command == 'make-plan':
            cfg.update(trajectory_action=args.trajectory_action or (
                '/joint_trajectory_with_para' if args.motion_backend == 'arms'
                else DEFAULTS['trajectory_action']),
                fsm_state_topic=args.fsm_state_topic, fsm_command_topic=args.fsm_command_topic)
        destination = args.output if args.command == 'make-plan' else args.output_dir
        if Path(destination).exists():
            raise ValueError(f'Output already exists: {destination}')
        if len(set(args.joints)) != len(args.joints):
            raise ValueError('Duplicate joint names')
        if 'filtered' in args.wrench_topic.lower():
            raise ValueError('Use the raw wrench topic, not wrench_filtered')
        if args.command == 'make-plan':
            generate_poses(np.zeros(len(args.joints)), args.joints, args.sweep_joints, args.angle_deg)
        elif args.poses < 6:
            raise ValueError('At least six manual poses required')
        cfg['gravity_vector'] = vector(args.gravity_vector, 3, 'gravity_vector').tolist()
        if np.linalg.norm(cfg['gravity_vector']) < 1e-3:
            raise ValueError('gravity_vector cannot be zero')
    else:
        cfg = load_plan(args.plan)
        if not args.execute:
            print(yaml.safe_dump(cfg, sort_keys=False))
            print('Plan validated; no ROS connection or motion. Add --execute to collect.')
            return
        if Path(args.output_dir).exists():
            raise ValueError('Output directory already exists; choose a new directory')
    import rclpy
    from rclpy.signals import SignalHandlerOptions
    # Keep ROS alive during Python's KeyboardInterrupt so finally can cancel/HOLD.
    rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
    node = make_ros_node(cfg, enable_motion=args.command == 'run')
    try:
        cfg['sensor_frame'] = node.ready()
        if args.command == 'make-plan':
            cfg['poses'] = generate_poses(node.q, args.joints, args.sweep_joints, args.angle_deg)
            validate_steps(node.q, cfg)
            cfg['schema_version'] = 2
            with open(args.output, 'x', encoding='utf-8') as f:
                yaml.safe_dump(cfg, f, sort_keys=False)
            print(f'Saved nine poses to {args.output}; no motion commands sent.')
            return
        if args.command == 'run':
            validate_steps(node.q, cfg)
        directory = Path(args.output_dir)
        directory.mkdir(parents=True, exist_ok=False)
        (directory / 'plan.yaml').write_text(yaml.safe_dump(cfg, sort_keys=False))
        rows = []
        try:
            with open(directory / 'samples.csv', 'x', newline='', encoding='utf-8') as stream:
                writer = csv.DictWriter(stream, fieldnames=COLUMNS)
                writer.writeheader()
                stream.flush()
                node.start_motion()
                count = args.poses if args.command == 'collect' else len(cfg['poses'])
                for i in range(count):
                    if args.command == 'collect':
                        if args.panel:
                            wait_panel_sample(node, i+1, count)
                        else:
                            answer = input(f'姿态 {i+1}/{count}：使用外部控制器摆好姿态并保持静止，回车采样（q 结束）：')
                            if answer.strip().lower() == 'q':
                                raise KeyboardInterrupt()
                        node.ready()
                        target = node.q.copy()
                    else:
                        node.get_logger().info(f'Pose {i+1}/{count}: moving, then static sampling')
                        target = np.array(cfg['poses'][i], dtype=float)
                        node.move(target)
                    rows.extend(node.collect(target, i, writer, stream))
                    if args.command == 'collect' and args.panel:
                        panel_event('sampled', pose=i+1, total=count, samples=len(rows))
            result = fit_rows(rows, cfg['max_condition'], cfg['max_force_rms'], cfg['max_torque_rms'])
            save_result(result, directory / 'result.yaml')
            if args.command == 'collect' and args.panel:
                panel_event('result', **result)
        except BaseException as exc:
            (directory / 'failure.txt').write_text(f'{type(exc).__name__}: {exc}\n')
            raise
    finally:
        try:
            node.stop()
        finally:
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest='command', required=True)
    acquisition = argparse.ArgumentParser(add_help=False)
    acquisition.add_argument('--joints', nargs='+', required=True, help='Exact names of the selected arm joints')
    acquisition.add_argument('--wrench-topic', required=True, help='Raw WrenchStamped topic, not filtered')
    acquisition.add_argument('--gravity-frame', required=True, help='Fixed frame with known gravity direction')
    acquisition.add_argument('--gravity-vector', type=float, nargs=3, default=[0., 0., -9.81])
    acquisition.add_argument('--joint-state-topic', default='/joint_states')
    plan = sub.add_parser('make-plan', parents=[acquisition],
                          help='Read current joints and save nine poses; never move')
    plan.add_argument('--sweep-joints', nargs=2, required=True, help='Two joints that tilt the sensor')
    plan.add_argument('--angle-deg', type=float, default=15.0)
    plan.add_argument('--fsm-state-topic', default='/fsm_state')
    plan.add_argument('--output', default='payload_poses.yaml')
    plan.add_argument('--motion-backend', choices=['follow_joint_trajectory', 'arms'],
                      default='follow_joint_trajectory')
    plan.add_argument('--trajectory-action', default=None)
    plan.add_argument('--fsm-command-topic', default='/fsm_command')
    collect = sub.add_parser('collect', parents=[acquisition],
                             help='Manual pose sampling: never sends motion or FSM commands')
    collect.add_argument('--poses', type=int, default=9)
    collect.add_argument('--output-dir', default='payload_identification')
    collect.add_argument('--panel', action='store_true', help='RViz JSON/stdin interface; requires COMPLIANCE with S=0')
    collect.add_argument('--compliance-status-topic', default='/compliance_force_status')
    collect.add_argument('--use-sim-time', action='store_true')
    run = sub.add_parser('run', help='Validate a plan; --execute enables real robot motion')
    run.add_argument('--plan', required=True)
    run.add_argument('--output-dir', default='payload_identification')
    run.add_argument('--execute', action='store_true')
    fit = sub.add_parser('fit', help='Refit saved CSV without ROS or robot motion')
    fit.add_argument('--input', required=True)
    fit.add_argument('--output', default='payload_result.yaml')
    fit.add_argument('--max-condition', type=float, default=1000.)
    fit.add_argument('--max-force-rms', type=float, default=0.5)
    fit.add_argument('--max-torque-rms', type=float, default=0.05)
    args = parser.parse_args(argv)
    try:
        if args.command == 'fit':
            for name in ('max_condition', 'max_force_rms', 'max_torque_rms'):
                if not np.isfinite(getattr(args, name)) or getattr(args, name) <= 0:
                    raise ValueError(f'{name} must be positive and finite')
            with open(args.input, newline='', encoding='utf-8') as f:
                result = fit_rows(list(csv.DictReader(f)), args.max_condition,
                                  args.max_force_rms, args.max_torque_rms)
            save_result(result, args.output)
        else:
            run_live(args)
    except KeyboardInterrupt:
        print('Interrupted; collection stopped.', file=sys.stderr)
        return 130
    except (ValueError, RuntimeError, TimeoutError, OSError, KeyError, TypeError, EOFError, ImportError) as exc:
        print(f'ERROR: {exc}', file=sys.stderr)
        return 1
    return 0


if __name__ == '__main__':
    def stop_on_signal(signum, frame):
        raise KeyboardInterrupt()
    signal.signal(signal.SIGTERM, stop_on_signal)
    sys.exit(main())
