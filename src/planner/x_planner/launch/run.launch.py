import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # 获取包路径
    pkg_share = get_package_share_directory('x_planner')
    
    # 参数文件路径
    param_file = os.path.join(pkg_share, 'config', 'param.yaml')
    
    # 创建节点
    planner_node = Node(
        package='x_planner',
        executable='main',  # 你的可执行文件名
        name='x_planner_node',  # 节点名，必须和 YAML 中的命名空间匹配
        output='screen',
        parameters=[param_file],  # 加载参数文件
        # 可以重映射话题
        remappings=[
            # ('/old_topic', '/new_topic'),
        ]
    )
    
    return LaunchDescription([
        planner_node,
    ])