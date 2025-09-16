#!/usr/bin/env python3

import os
import re
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition

def parse_task_info(task_file_path):
    """解析task.info文件，提取dual_arm_mode和frame_id信息"""
    dual_arm_mode = False
    frame_id = "world"
    
    if not os.path.exists(task_file_path):
        print(f"[WARN] Task file not found: {task_file_path}")
        return dual_arm_mode, frame_id
    
    try:
        with open(task_file_path, 'r') as file:
            content = file.read()
        
        # 检查是否有dualArmMode配置
        dual_arm_match = re.search(r'dualArmMode\s+true', content)
        if dual_arm_match:
            dual_arm_mode = True
            print(f"[INFO] Detected dual arm mode from task file")
        
        # 检查是否有eeFrame1（双臂机器人的第二个末端执行器）
        ee_frame1_match = re.search(r'eeFrame1\s+"([^"]+)"', content)
        if ee_frame1_match:
            dual_arm_mode = True
            print(f"[INFO] Detected dual arm mode from eeFrame1: {ee_frame1_match.group(1)}")
        
        # 提取baseFrame
        base_frame_match = re.search(r'baseFrame\s+"([^"]+)"', content)
        if base_frame_match:
            frame_id = base_frame_match.group(1)
            print(f"[INFO] Detected base frame: {frame_id}")
        
        print(f"[INFO] Parsed task file - dual_arm_mode: {dual_arm_mode}, frame_id: {frame_id}")
        
    except Exception as e:
        print(f"[ERROR] Failed to parse task file {task_file_path}: {e}")
    
    return dual_arm_mode, frame_id

def launch_setup(context, *args, **kwargs):
    """Launch setup function using OpaqueFunction"""
    robot_name = context.launch_configurations['robot']
    task_file_path = context.launch_configurations.get('task_file', '')
    
    # 检查是否提供了task_file路径
    if not task_file_path:
        print(f"[ERROR] No task_file provided for robot '{robot_name}'")
        return []
    
    print(f"[INFO] Using provided task file: {task_file_path}")
    
    # 解析task.info文件
    dual_arm_mode, frame_id = parse_task_info(task_file_path)
    
    # 创建ArmsTargetManager节点
    arms_target_manager_node = Node(
        package='arms_target_manager',
        executable='arms_target_manager_node',
        name='arms_target_manager',
        output='screen',
        parameters=[
            {'topic_prefix': robot_name},
            {'dual_arm_mode': dual_arm_mode},
            {'frame_id': frame_id},
        ],
    )
    
    return [arms_target_manager_node]

def generate_launch_description():
    # 声明启动参数
    robot_name_arg = DeclareLaunchArgument(
        'robot',
        default_value='cr5',
        description='Robot name (arx5, cr5, etc.)'
    )
    
    task_file_arg = DeclareLaunchArgument(
        'task_file',
        default_value='',
        description='Path to task.info file'
    )

    return LaunchDescription([
        robot_name_arg,
        task_file_arg,
        OpaqueFunction(function=launch_setup),
    ])
