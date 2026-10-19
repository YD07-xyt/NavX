from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_map_to_world',
        arguments=[
            '--x', '0.0',
            '--y', '0.0',
            '--z', '0.0',
            '--roll', '0.0',
            '--pitch', '0.0',
            '--yaw', '0.0',
            '--frame-id', 'map',
            '--child-frame-id', 'world'
        ],
        output='screen'
    )
    slam_toolbox_node=Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node', # 或 sync_slam_toolbox_node
            name='slam_toolbox',
            output='screen',
            parameters=[
                'src/bringup/config/mapping.yaml' # 直接传入 YAML 文件路径
            ],
        )
    return LaunchDescription([
        static_tf_node,
        slam_toolbox_node
    ])