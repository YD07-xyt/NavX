from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            name='scanner', default_value='scanner',
            description='Namespace for sample topics'
        ),
        Node(
            package='pointcloud_to_laserscan', executable='pointcloud_to_laserscan_node',
            remappings=[
                ('cloud_in', '/cloud_preprocess/obstacles'),   # 直接使用全局话题
                ('scan', '/scan')   
                ],
            parameters=[{
                'target_frame': 'world',
                'transform_tolerance': 0.1,
                'min_height': -100.0,
                'max_height': 100.0,
                'angle_min': -3.14159,
                'angle_max': 3.14159,
                'angle_increment': 0.0087,
                'scan_time': 0.3333,
                'range_min': 0.1,
                'range_max': 10.0,
                'use_inf': True,
                'inf_epsilon': 1.0
            }],
            name='pointcloud_to_laserscan'
        )
    ])
