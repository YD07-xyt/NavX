import os
import launch.logging
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch.conditions import IfCondition
from launch_ros.actions import Node

def generate_launch_description():
    
    pkg_super_lio = get_package_share_directory('super_lio')
    config_yaml = os.path.join(pkg_super_lio, 'config', 'livox_360.yaml')
    super_lio_node = Node(
        package='super_lio',
        executable='super_lio_node',
        name='super_lio_node',
        output='screen',
        parameters=[config_yaml],
        arguments=['--ros-args', '--log-level', 'info']
    )


    # 获取功能包路径
    ma_nav_pkg_share = get_package_share_directory('ma_nav')
    
    # 默认参数文件路径（可选，如果不存在可注释）
    ma_nav_params_file = os.path.join(ma_nav_pkg_share, 'config', 'planner.yaml')
    
    # 全局规划节点
    ma_nav_node = Node(
        package='ma_nav',
        executable='ma_nav',
        name='ma_nav_node',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'planner_params_path': ma_nav_params_file
        }]
    )

    # 启动 RViz2 可视化
    rviz_config = os.path.join(ma_nav_pkg_share, 'config', 'nav.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen'
    )
    ld = LaunchDescription()
    ld.add_action(super_lio_node)
    ld.add_action(ma_nav_node)
    ld.add_action(rviz_node)

    return ld