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

    # pkg_terrain_analysis = get_package_share_directory('terrain_analysis_map')
    # terrain_analysis_config_yaml = os.path.join(pkg_terrain_analysis, 'config', 'param.yaml')
  
    # terrain_analysis_node = Node(
    #     package="terrain_analysis_map",
    #     executable="terrain_analysis_map_node",
    #     name="terrain_analysis_map_node",
    #     output="screen",
    #     respawn=True,
    #     respawn_delay=2.0,
    #     #arguments=["--ros-args", "--log-level", log_level],
    #     parameters=[terrain_analysis_config_yaml],
    # )

    # 获取功能包路径
    rog_map_pkg_share = get_package_share_directory('rog_map')
    rog_map_params_file = os.path.join(rog_map_pkg_share, 'config', 'rog_map.yaml')
    rog_map_node=Node(
            package='rog_map',
            executable='rog_map_node',
            name='rog_map_node',
            output='screen',
            parameters=[{'config_path': rog_map_params_file}]
        )
    ld = LaunchDescription()

    # 获取功能包路径
    gcopter_pkg_share = get_package_share_directory('gcopter')
    
    # 默认参数文件路径（可选，如果不存在可注释）
    gcopter_params_file = os.path.join(gcopter_pkg_share, 'config', 'global_planning.yaml')
    
    # 全局规划节点
    global_planner_node = Node(
        package='gcopter',
        executable='global_planning',
        name='global_planning_node',
        output='screen',
        emulate_tty=True,
        parameters=[gcopter_params_file],
    )

    ld.add_action(super_lio_node)
    #ld.add_action(terrain_analysis_node)
    ld.add_action(global_planner_node)
    ld.add_action(rog_map_node)

    return ld