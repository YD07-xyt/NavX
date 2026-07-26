#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 获取功能包路径
    pkg_share = get_package_share_directory('relocation')
    
    # 默认参数文件路径（可选，如果不存在可注释）
    default_params_file = os.path.join(pkg_share, 'config', 'param.yaml')
    
    # 声明启动参数：允许外部传入参数文件路径
    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=default_params_file,
        description='Path to the YAML parameter file'
    )
    
    # 全局规划节点
    global_planner_node = Node(
        package='relocation',
        executable='relocation_node',
        name='relocation_node',
        output='screen',
        emulate_tty=True,
        parameters=[LaunchConfiguration('params_file')]
    )
    
    return LaunchDescription([
        params_file_arg,
        global_planner_node,
    ])