# Copyright 2025 Lihan Chen
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition  # 新增 UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # 声明一个 'slam' 参数，默认值为 'False'
    declare_slam_cmd = DeclareLaunchArgument(
        'slam',
        default_value='False',
        description='If True, small_gicp will be disabled.'
    )
    
    # 将 'slam' 参数的值加载为一个 LaunchConfiguration 对象
    slam = LaunchConfiguration('slam')

    # 关键改动1：remappings 保留，但后续通过namespace统一TF
    remappings = [("/tf", "tf"), ("/tf_static", "tf_static")]

    node = Node(
        package="small_gicp_relocalization",
        executable="small_gicp_relocalization_node",
        namespace="/red_standard_robot1",  # 关键改动2：统一到机器人命名空间
        output="screen",
        remappings=remappings,
        # 关键改动3：逻辑修正 - slam=False时才启动节点（UnlessCondition）
        condition=UnlessCondition(slam),
        parameters=[
            {
                "num_threads": 4,
                "num_neighbors": 10,
                "global_leaf_size": 0.25,
                "registered_leaf_size": 0.25,
                "max_dist_sq": 1.0,
                "map_frame": "map",          # 已在/red_standard_robot1命名空间下，无需加前缀
                "odom_frame": "odom",        # 同上
                "base_frame": "base_footprint",  # 关键改动4：指定base_frame为实际存在的坐标系
                "lidar_frame": "livox_frame",    # 补充：需替换为你实际的激光雷达坐标系（必填）
                "prior_pcd_file": "",            # 按需填写预加载的地图PCD文件路径
            }
        ],
    )

    # 将声明参数的动作和节点都添加到 LaunchDescription 中
    return LaunchDescription([
        declare_slam_cmd,
        node
    ])