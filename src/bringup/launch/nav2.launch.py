from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def generate_launch_description():
    # 参数文件路径
    params_file = PathJoinSubstitution([
        FindPackageShare('bringup'),   # 包名
        'config',
        'nav2_params.yaml'                  # 参数文件名
    ])

    # 使用 nav2_bringup 的 bringup_launch.py
    nav2_bringup_launch = PathJoinSubstitution([
        FindPackageShare('nav2_bringup'),
        'launch',
        'bringup_launch.py'
    ])
    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        parameters=[params_file],
        output='screen'
    )
    return LaunchDescription([
        slam_node,
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav2_bringup_launch),
            launch_arguments={
                'params_file': params_file,
                'use_sim_time': 'false',      # 根据实际情况改为 'true' 或 'false'
                'autostart': 'true',           # 自动启动所有生命周期节点
                'map': '' 
            }.items()
        )
    ])