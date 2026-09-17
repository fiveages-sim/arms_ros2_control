"""Optional motion adapters for identify_payload; all ROS imports are lazy.

The estimator and manual sampler do not import robot-specific messages.
"""
import time

class ActionMotion:
    def __init__(self, node, cfg, action_type):
        from rclpy.action import ActionClient
        self.node, self.cfg = node, cfg
        self.client = ActionClient(node, action_type, cfg['trajectory_action'])
        self.goal = self.send_future = self.result_future = None

    def ready(self):
        pass

    def guard(self):
        pass

    def start(self):
        self.node.wait(self.client.server_is_ready, self.cfg['wait_timeout'],
                       'Trajectory action server unavailable')

    def execute(self, goal, duration):
        from action_msgs.msg import GoalStatus
        self.send_future = self.client.send_goal_async(goal)
        self.node.wait(self.send_future.done, self.cfg['wait_timeout'],
                       'Goal acceptance timeout', self.node.guard_motion)
        self.goal = self.send_future.result()
        if not self.goal.accepted:
            raise RuntimeError('Joint trajectory goal rejected')
        self.result_future = self.goal.get_result_async()
        self.node.wait(self.result_future.done, max(self.cfg['wait_timeout'], duration*3+10),
                       'Motion completion timeout', self.node.guard_motion)
        response = self.result_future.result()
        if response.status != GoalStatus.STATUS_SUCCEEDED or not self.success(response.result):
            raise RuntimeError(f'Motion failed: {response.result}')
        self.goal = self.send_future = self.result_future = None

    def stop(self):
        import rclpy

        def wait_closing(predicate, message):
            # Cleanup must keep working after malformed sensor messages set node.error.
            deadline = time.monotonic() + 3.
            while not predicate():
                if time.monotonic() >= deadline or not rclpy.ok():
                    raise RuntimeError(message)
                rclpy.spin_once(self.node, timeout_sec=0.05)

        # Never cancel other clients' goals. Resolve a pending acceptance before canceling ours.
        if self.send_future is None:
            return
        wait_closing(self.send_future.done, 'Cannot confirm pending goal acceptance for cancellation')
        handle = self.send_future.result()
        if handle is None or not handle.accepted:
            return
        result = self.result_future or handle.get_result_async()
        if result.done():
            return
        cancel = handle.cancel_goal_async()
        wait_closing(cancel.done, 'Action cancellation response timeout')
        if not cancel.result().goals_canceling and not result.done():
            raise RuntimeError('Controller did not accept trajectory cancellation')
        wait_closing(result.done, 'Controller did not confirm trajectory termination')


class FollowJointTrajectoryMotion(ActionMotion):
    def __init__(self, node, cfg):
        from control_msgs.action import FollowJointTrajectory
        super().__init__(node, cfg, FollowJointTrajectory)

    def move(self, target, duration):
        from builtin_interfaces.msg import Duration
        from control_msgs.action import FollowJointTrajectory
        from trajectory_msgs.msg import JointTrajectoryPoint
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = self.cfg['joint_names']
        start = JointTrajectoryPoint(positions=self.node.q.tolist(),
                                     velocities=[0.] * len(target))
        end = JointTrajectoryPoint(positions=target.tolist(), velocities=[0.] * len(target))
        ns = int(round(duration*1e9))
        end.time_from_start = Duration(sec=ns//1000000000, nanosec=ns%1000000000)
        goal.trajectory.points = [start, end]
        self.execute(goal, duration)

    @staticmethod
    def success(result):
        return result.error_code == 0


class ArmsMotion(ActionMotion):
    """Repository MOVEJ adapter shared by controllers exposing this FSM/action."""
    def __init__(self, node, cfg):
        from arms_ros2_control_msgs.action import JointTrajectory
        from rclpy.qos import QoSProfile, DurabilityPolicy
        from std_msgs.msg import Int32
        super().__init__(node, cfg, JointTrajectory)
        self.state = None
        self.state_sequence = 0
        self.started = False
        self.subscription = node.create_subscription(
            Int32, cfg['fsm_state_topic'], self.on_state,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.publisher = node.create_publisher(Int32, cfg['fsm_command_topic'], 1)

    def on_state(self, msg):
        self.state = msg.data
        self.state_sequence += 1

    def ready(self):
        self.node.wait(lambda: self.state is not None, self.cfg['wait_timeout'], 'Waiting for FSM state')
        if self.state != 2:
            raise RuntimeError(f'Start in HOLD (2), current FSM state={self.state}')

    def guard(self):
        if self.state != 4:
            raise RuntimeError(f'MOVEJ was interrupted, FSM state={self.state}')

    def start(self):
        from std_msgs.msg import Int32
        super().start()
        self.node.wait(lambda: self.publisher.get_subscription_count() > 0,
                       self.cfg['wait_timeout'], 'FSM command subscriber unavailable')
        self.started = True
        self.publisher.publish(Int32(data=4))
        self.node.wait(lambda: self.state == 4, self.cfg['wait_timeout'], 'MOVEJ transition timed out')

    def move(self, target, duration):
        from arms_ros2_control_msgs.action import JointTrajectory
        from arms_ros2_control_msgs.msg import JointWaypoint
        waypoint = JointWaypoint()
        waypoint.position, waypoint.velocity = target.tolist(), [0.] * len(target)
        waypoint.max_velocity = [float(self.cfg['max_velocity'])] * len(target)
        waypoint.max_acceleration = [float(self.cfg['max_acceleration'])] * len(target)
        waypoint.max_jerk = [float(self.cfg['max_jerk'])] * len(target)
        waypoint.time_mode, waypoint.total_time = True, float(duration)
        self.execute(JointTrajectory.Goal(joint_names=self.cfg['joint_names'], waypoints=[waypoint]), duration)

    @staticmethod
    def success(result):
        return result.success

    def stop(self):
        if not self.started:
            return
        import rclpy
        from std_msgs.msg import Int32
        sequence = self.state_sequence
        # HOLD also handles delayed goal acceptance; this adapter owns the FSM session.
        self.publisher.publish(Int32(data=2))
        if self.goal is not None and self.goal.accepted:
            self.goal.cancel_goal_async()
        deadline = time.monotonic() + 3.
        while time.monotonic() < deadline and rclpy.ok():
            rclpy.spin_once(self.node, timeout_sec=0.05)
            if self.state == 2 and self.state_sequence > sequence:
                return
        raise RuntimeError('HOLD acknowledgement timeout; check controller state')


def create_motion(node, cfg):
    kind = cfg['motion_backend']
    if kind == 'arms':
        return ArmsMotion(node, cfg)
    if kind == 'follow_joint_trajectory':
        return FollowJointTrajectoryMotion(node, cfg)
    if kind == 'manual':
        return None
    raise ValueError(f'Unknown motion_backend: {kind}')
