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

    pkg_terrain_analysis = get_package_share_directory('terrain_analysis')
    terrain_analysis_config_yaml = os.path.join(pkg_terrain_analysis, 'config', 'param.yaml')
    terrain_analysis_node = Node(
        package="terrain_analysis",
        executable="terrainAnalysis",
        name="terrain_analysis",
        output="screen",
        respawn=True,
        respawn_delay=2.0,
        #arguments=["--ros-args", "--log-level", log_level],
        parameters=[terrain_analysis_config_yaml],
    )

    pkg_terrain_analysis_ext = get_package_share_directory('terrain_analysis_ext')
    terrain_analysis_ext_config_yaml = os.path.join(pkg_terrain_analysis_ext, 'config', 'param.yaml')
    terrain_analysis_ext_node = Node(
        package="terrain_analysis_ext",
        executable="terrainAnalysisExt",
        name="terrain_analysis_ext",
        output="screen",
        respawn=True,
        respawn_delay=2.0,
        parameters=[terrain_analysis_ext_config_yaml],
    )
    
    ld = LaunchDescription()
    ld.add_action(super_lio_node)
    ld.add_action(terrain_analysis_node)
    ld.add_action(terrain_analysis_ext_node)

    return ld