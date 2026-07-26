import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


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

    pkg_share = get_package_share_directory('cloud_preprocess')
    params_file = os.path.join(pkg_share, 'config', 'cloud_preprocess.yaml')
    cloud_preprocess_node= Node(
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
        )

    pointcloud2scan_node=Node(
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

    # 参数文件路径
    params_file = PathJoinSubstitution([
        FindPackageShare('bringup'),
        'config',
        'nav2_params.yaml'
    ])

    # ---- SLAM（建图） ----
    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        parameters=[params_file],
        output='screen'
    )

    # ---- 导航核心节点 ----
    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        parameters=[params_file],
        remappings=[
            # 确保速度指令发送到底盘对应的 topic
            ('cmd_vel', '/cmd_vel_chassis')
        ],
        output='screen'
    )

    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        parameters=[params_file],
        output='screen'
    )

    smoother_server = Node(
        package='nav2_smoother',
        executable='smoother_server',
        name='smoother_server',
        parameters=[params_file],
        output='screen'
    )

    behavior_server = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        parameters=[params_file],
        output='screen'
    )

    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        parameters=[params_file],
        output='screen'
    )

    waypoint_follower = Node(
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        name='waypoint_follower',
        parameters=[params_file],
        output='screen'
    )

    velocity_smoother = Node(
        package='nav2_velocity_smoother',
        executable='velocity_smoother',
        name='velocity_smoother',
        parameters=[params_file],
        output='screen'
    )

    # ---- 生命周期管理器（自动激活所有导航节点） ----
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'autostart': True,
            'node_names': [
                'controller_server',
                'planner_server',
                'smoother_server',
                'behavior_server',
                'bt_navigator',
                'waypoint_follower',
                'velocity_smoother'
            ]
        }]
    )

    # 如果需要保存地图，可以单独启动 map_saver（按需，这里不加）
    # map_saver 会在 /map 话题上有更新时保存，也可以手动调用服务
    # map_saver = Node(
    #     package='nav2_map_server',
    #     executable='map_saver',
    #     name='map_saver',
    #     parameters=[params_file],
    #     output='screen'
    # )
    return LaunchDescription([
        static_tf_node,
        super_lio_node,
        cloud_preprocess_node,
        pointcloud2scan_node,
        slam_node,
        controller_server,
        planner_server,
        smoother_server,
        behavior_server,
        bt_navigator,
        waypoint_follower,
        velocity_smoother,
        lifecycle_manager
    ])