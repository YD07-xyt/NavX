from launch_ros.actions import Node
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time", default="True")

    # 获取包路径并拼接 yaml
    pkg_share = get_package_share_directory('fake_vel_transform')
    param_file = os.path.join(pkg_share, 'config', 'params.yaml')

    fake_vel_transform_node = Node(
        package="fake_vel_transform",
        executable="fake_vel_transform_node",
        output="screen",
        parameters=[param_file],
    )

    return LaunchDescription([fake_vel_transform_node])