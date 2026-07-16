"""Launch cloud_preprocess 点云预处理节点。

话题名在 config/cloud_preprocess.yaml 的 input_cloud_topic / *_topic 中配置，
按需改为真实设备话题即可。节点仅订阅 sensor_msgs/msg/PointCloud2。
"""
from launch import LaunchDescription
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('cloud_preprocess')
    params_file = os.path.join(pkg_share, 'config', 'cloud_preprocess.yaml')
    return LaunchDescription([
        Node(
            package='cloud_preprocess',
            executable='cloud_preprocess_node',
            name='cloud_preprocess_node',
            output='screen',
            parameters=[{'use_sim_time': False}, params_file],
            remappings=[
                ('~/input_cloud', '/livox/lidar/pointcloud'),
                ('~/obstacles', '/cloud_preprocess/obstacles'),
                ('~/ground', '/cloud_preprocess/ground'),
                ('~/inflated', '/cloud_preprocess/inflated'),
            ],
        ),
    ])
