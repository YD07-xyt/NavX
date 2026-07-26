import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 获取包路径
    package_name = 'serial'
    config_dir = os.path.join(get_package_share_directory(package_name), 'config')
    default_param_file = os.path.join(config_dir, 'param.yaml')
    
    return LaunchDescription([
        # 声明参数文件参数
        DeclareLaunchArgument(
            'param_file',
            default_value=default_param_file,
            description='Path to parameter file'
        ),
        
        # 启动节点
        Node(
            package=package_name,
            executable='serial_node',
            name='serial_node',
            parameters=[LaunchConfiguration('param_file')],
            output='screen',
        )
    ])