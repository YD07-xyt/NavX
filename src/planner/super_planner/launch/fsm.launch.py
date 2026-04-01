from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='super_planner',  # 替换为实际package名
            executable='fsm_ros2_node',  # 需要先编译生成的可执行文件
            name='fsm_node',
            namespace='',
            parameters=[
                {'config_path': '/home/xyt/code/nav/NavX/src/planner/super_planner/config/static_high_speed.yaml'},  # 配置文件路径
            ],
            output='screen',
        )
    ])