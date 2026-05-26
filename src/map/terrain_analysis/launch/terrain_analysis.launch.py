import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import LoadComposableNodes, Node
from launch_ros.descriptions import ComposableNode, ParameterFile
def generate_launch_description():
    namespace = LaunchConfiguration("namespace")
    #use_sim_time = LaunchConfiguration("use_sim_time")
    #autostart = LaunchConfiguration("autostart")
    #params_file = LaunchConfiguration("params_file")
    use_composition = LaunchConfiguration("use_composition")
    container_name = LaunchConfiguration("container_name")
    # Create our own temporary YAML files that include substitutions
    #param_substitutions = {"use_sim_time": use_sim_time, "autostart": autostart}
    
    # configured_params = ParameterFile(
    #     RewrittenYaml(
    #         source_file=params_file,
    #         root_key=namespace,
    #         #param_rewrites=param_substitutions,
    #         convert_types=True,
    #     ),
    #     allow_substs=True,
    # )
    pkg_terrain_analysis = get_package_share_directory('terrain_analysis')
    terrain_analysis_config_yaml = os.path.join(pkg_terrain_analysis, 'config', 'param.yaml')
  
    start_terrain_analysis_cmd = Node(
        package="terrain_analysis",
        executable="terrainAnalysis",
        name="terrain_analysis",
        output="screen",
        respawn=True,
        respawn_delay=2.0,
        #arguments=["--ros-args", "--log-level", log_level],
        parameters=[terrain_analysis_config_yaml],
    )
    ld = LaunchDescription()
    ld.add_action(start_terrain_analysis_cmd)
    return ld
