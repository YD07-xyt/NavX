from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'config_path',
            default_value='/home/xyt/code/nav/NavX/src/map/rog_map/config/rog_map.yaml',
            description='Path to config file'
        ),
        
        Node(
            package='rog_map',
            executable='rog_map_node',
            name='rog_map_node',
            output='screen',
            parameters=[{
                'config_path': LaunchConfiguration('config_path')
            }]
        )
    ])