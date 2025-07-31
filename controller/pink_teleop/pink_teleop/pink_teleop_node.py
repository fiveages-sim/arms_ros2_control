#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Pose

import numpy as np
import qpsolvers
from pink import solve_ik
from pink.configuration import Configuration
from pinocchio.robot_wrapper import RobotWrapper
from pink.tasks import FrameTask, PostureTask
import tempfile
import os
from sensor_msgs.msg import JointState
import pinocchio as pin
from pink.utils import custom_configuration_vector

class PinkTeleopNode(Node):
    def __init__(self):
        super().__init__('pink_teleop_node')
        
        # 从参数服务器获取配置
        self.declare_parameters(
            namespace='',
            parameters=[
                ('linear_scale', 0.1),
                ('angular_scale', 0.1),
                ('end_effector_link', 'end_effector'),
                ('urdf_path', ''),  # 新增URDF路径参数
                ('robot_description', ''),  # 新增URDF路径参数
                ('joint_states_topic', 'joint_states'),  # 新增
                ('joint_commands_topic', 'joint_commands'),  # 新增
                ('default_joint_positions', [0.0]),  # 新增默认关节位置参数
                ('gripper_joint_names', ['gripper_joint']),  # 修改为夹爪关节名称列表
                ('gripper_open_positions', [0.0]),  # 新增：夹爪打开位置
                ('gripper_close_positions', [1.0]),  # 新增：夹爪关闭位置
                ('max_joint_velocity', 0.5),  # 新增：最大关节速度限制
            ]
        )
        
        # 获取参数
        self.linear_scale = self.get_parameter('linear_scale').value
        self.angular_scale = self.get_parameter('angular_scale').value
        self.end_effector_link = self.get_parameter('end_effector_link').value
        self.urdf_path = self.get_parameter('urdf_path').value
        self.joint_states_topic = self.get_parameter('joint_states_topic').value
        self.joint_commands_topic = self.get_parameter('joint_commands_topic').value
        self.gripper_joint_names = self.get_parameter('gripper_joint_names').value
        self.gripper_open_positions = self.get_parameter('gripper_open_positions').value
        self.gripper_close_positions = self.get_parameter('gripper_close_positions').value
        self.max_joint_velocity = self.get_parameter('max_joint_velocity').value
        default_positions = self.get_parameter('default_joint_positions').value  # 获取默认位置参数
        
        # 验证夹爪参数
        if len(self.gripper_joint_names) != len(self.gripper_open_positions) or \
           len(self.gripper_joint_names) != len(self.gripper_close_positions):
            raise ValueError("gripper_joint_names, gripper_open_positions, and gripper_close_positions must have the same length")
        
        # 初始化标志
        self.joint_state_updated = False
        self.joy_input_updated = False  # 新增：手柄输入更新标志
        self.last_joy_msg = None  # 新增：存储最新的手柄消息
        self.gripper_closed = False  # 新增：夹爪状态标志
        self.last_button2_state = False  # 新增：记录上一次按钮2的状态
        
        # 初始化机器人模型
        if self.urdf_path:
            self.robot_wrapper = RobotWrapper.BuildFromURDF(self.urdf_path)
        else:
            self.robot_description = self.get_parameter('robot_description').value
            with tempfile.NamedTemporaryFile(mode='w', suffix='.urdf', delete=False) as f:
                f.write(self.robot_description)
                self.temp_urdf_path = f.name
            self.robot_wrapper = RobotWrapper.BuildFromURDF(self.temp_urdf_path)
        
        # 获取机器人关节名称列表（排除universe）
        joint_names = [name for name in self.robot_wrapper.model.names if name != 'universe']
        
        # 创建默认位置字典
        default_positions_dict = {}
        if len(default_positions) == 1 and default_positions[0] == 0.0:  # 如果没有提供参数
            # 使用robot_wrapper.q0作为默认值
            for i, name in enumerate(joint_names):
                default_positions_dict[name] = self.robot_wrapper.q0[i]
        else:
            # 使用提供的参数，不足的补0
            for i, name in enumerate(joint_names):
                if i < len(default_positions):
                    default_positions_dict[name] = default_positions[i]
                else:
                    default_positions_dict[name] = self.robot_wrapper.q0[i]
        
        # 使用custom_configuration_vector创建默认配置
        q_default = custom_configuration_vector(
            self.robot_wrapper,
            **default_positions_dict
        )
        
        # 更新配置
        self.pink_configuration = Configuration(
            self.robot_wrapper.model, 
            self.robot_wrapper.data, 
            q_default
        )

        # 设置任务
        self.end_effector_task = FrameTask(
            self.end_effector_link,
            position_cost=1.0,
            orientation_cost=1.0,
            lm_damping=15.0,
        )
        self.posture_task = PostureTask(
            cost=1e-3,
        )

        # 添加关节限位任务
        self.joint_limit_task = PostureTask(
            cost=1e-2,  # 设置一个较大的权重以确保关节限位
        )

        self.tasks = [
            self.end_effector_task,
            self.posture_task,
            self.joint_limit_task
        ]

        for task in self.tasks:
            task.set_target_from_configuration(self.pink_configuration)

        self.solver = qpsolvers.available_solvers[0]
        if "osqp" in qpsolvers.available_solvers:
            self.solver = "osqp"
        
        # 创建订阅者和发布者
        self.joy_sub = self.create_subscription(Joy, 'joy', self.joy_callback, 10)
        
        # 创建定时器用于发布目标位姿
        self.timer = self.create_timer(0.01, self.timer_callback)  # 100Hz
        
        # 控制参数
        self.dt = 0.1  # 时间步长
        
        # 订阅 joint_states
        self.current_joint_state = None
        self.joint_state_sub = self.create_subscription(
            JointState, self.joint_states_topic, self.joint_state_callback, 10
        )
        # 发布 joint_commands
        self.joint_command_pub = self.create_publisher(
            JointState, self.joint_commands_topic, 10
        )
        
        # 发布初始关节位置
        joint_cmd = JointState()
        joint_cmd.header.stamp = self.get_clock().now().to_msg()
        joint_cmd.name = [str(name) for name in joint_names]
        joint_cmd.position = q_default.tolist()
        self.joint_command_pub.publish(joint_cmd)
        
        self.get_logger().info('Pink Teleop Node has been initialized')

    def __del__(self):
        """清理临时文件"""
        if hasattr(self, 'temp_urdf_path'):
            try:
                os.unlink(self.temp_urdf_path)
            except:
                pass

    def joy_callback(self, msg):
        """处理游戏手柄输入"""
        # 只存储最新的手柄消息，不立即处理
        self.last_joy_msg = msg
        self.joy_input_updated = True

    def joint_state_callback(self, msg):
        """处理关节状态更新"""
        self.current_joint_state = msg
        
        # 更新pink_configuration的当前状态
        if self.current_joint_state is not None:
            # 创建一个新的配置向量，大小与模型一致
            q = np.zeros(self.pink_configuration.model.nq)
            
            # 从joint_states更新关节位置
            model_names = [name for name in self.pink_configuration.model.names if name != 'universe']
            for i, name in enumerate(model_names):  # 从0开始，因为已经去掉了universe
                if name in self.current_joint_state.name:
                    idx = self.current_joint_state.name.index(name)
                    q[i] = self.current_joint_state.position[idx]
            
            # 更新pink_configuration
            self.pink_configuration.update(q)
            
            # 设置更新标志
            self.joint_state_updated = True
            
    def timer_callback(self):
        # 只在关节状态更新时计算逆运动学
        if not self.joint_state_updated:
            return
            
        # 处理手柄输入
        if self.joy_input_updated and self.last_joy_msg is not None:
            # 检查按钮2的状态变化
            current_button2_state = self.last_joy_msg.buttons[2]  # 按钮2的索引是1
            if current_button2_state and not self.last_button2_state:  # 检测到按钮2的上升沿
                self.gripper_closed = not self.gripper_closed  # 切换夹爪状态
                self.get_logger().info(f'Gripper state changed to: {"closed" if self.gripper_closed else "open"}')
            self.last_button2_state = current_button2_state

            # 在当前位置基础上更新目标位置
            end_effector_target = self.end_effector_task.transform_target_to_world
            
            # 位置控制
            end_effector_target.translation[0] += self.last_joy_msg.axes[1] * self.linear_scale # x
            end_effector_target.translation[1] += self.last_joy_msg.axes[0] * self.linear_scale # y
            end_effector_target.translation[2] += self.last_joy_msg.axes[4] * self.linear_scale # z

            # 姿态控制
            # 获取当前旋转矩阵
            current_rotation = end_effector_target.rotation
            
            # 计算旋转增量
            # 使用右摇杆控制旋转
            # axes[3] 控制绕Z轴旋转（偏航）
            # axes[5] 控制绕X轴旋转（俯仰）
            yaw_angle = self.last_joy_msg.axes[3] * self.angular_scale
            pitch_angle = -self.last_joy_msg.axes[6] * self.angular_scale
            roll_angle = self.last_joy_msg.axes[7] * self.angular_scale
            
            # 创建绕Z轴和X轴的旋转矩阵
            yaw_rotation = pin.AngleAxis(yaw_angle, np.array([0.0, 0.0, 1.0])).toRotationMatrix()
            pitch_rotation = pin.AngleAxis(pitch_angle, np.array([1.0, 0.0, 0.0])).toRotationMatrix()
            roll_rotation = pin.AngleAxis(roll_angle, np.array([0.0, 1.0, 0.0])).toRotationMatrix()
            
            # 组合旋转：先绕Z轴旋转，再绕X轴旋转
            new_rotation = current_rotation @ yaw_rotation @ pitch_rotation @ roll_rotation
            
            # 更新目标姿态
            end_effector_target.rotation = new_rotation

            # 发布关节指令
            joint_cmd = JointState()
            joint_cmd.header.stamp = self.get_clock().now().to_msg()
            joint_cmd.name = [str(name) for name in self.pink_configuration.model.names[1:] if name != 'universe']
            
            # 计算新的关节位置
            try:
                velocity = solve_ik(
                    self.pink_configuration,
                    self.tasks,
                    self.dt,
                    self.solver
                )
                
                # 限制关节速度
                max_velocity = self.max_joint_velocity
                velocity = np.clip(velocity, -max_velocity, max_velocity)
                
                Delta_q = velocity * self.dt
                new_q = self.pink_configuration.q + Delta_q
                
                # 确保关节角度在有效范围内
                for i in range(len(new_q)):
                    lower = self.robot_wrapper.model.lowerPositionLimit[i]
                    upper = self.robot_wrapper.model.upperPositionLimit[i]
                    new_q[i] = np.clip(new_q[i], lower, upper)
                
                # 设置夹爪位置
                for i, joint_name in enumerate(self.gripper_joint_names):
                    if joint_name in joint_cmd.name:
                        gripper_idx = joint_cmd.name.index(joint_name)
                        # 根据夹爪状态设置位置
                        new_q[gripper_idx] = self.gripper_close_positions[i] if self.gripper_closed else self.gripper_open_positions[i]
                
                joint_cmd.position = new_q.tolist()
                self.joint_command_pub.publish(joint_cmd)
                
            except Exception as e:
                self.get_logger().warn(f'IK solver failed: {str(e)}')

            self.joy_input_updated = False

def main(args=None):
    rclpy.init(args=args)
    node = PinkTeleopNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main() 