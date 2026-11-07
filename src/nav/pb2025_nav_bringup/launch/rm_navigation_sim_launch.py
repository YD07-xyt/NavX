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

"""仿真顶层入口：Gazebo（pb_rm_simulation）+ 导航栈。

与 rm_navigation_reality_launch.py 的区别：
- 不启动 livox_ros_driver2：雷达由 Gazebo 的 livox 插件直接发 CustomMsg（带 offset_time）
- 不启动 robot_state_publisher：由 pb_rm_simulation 用 URDF 提供 base_link -> livox_frame
- 不启动 base_link -> livox_frame 静态外参：避免和 URDF 重复
- 默认 use_sim_time=True、params_file=config/simulation/nav2_params.yaml

用法：
    ros2 launch pb2025_nav_bringup rm_navigation_sim_launch.py world:=RMUC2027
    ros2 launch pb2025_nav_bringup rm_navigation_sim_launch.py world:=RMUC slam:=False \
        map:=.../map/simulation/rmuc_2025.yaml prior_pcd_file:=.../pcd/simulation/rmuc_2025.pcd

注意：`world` 是 Gazebo 世界名（RMUC/RMUL/RMUL2026H/RMUC2027），
      `map`/`prior_pcd_file` 才使用 map/simulation 下的地图名，只在 slam:=False 时生效。
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution


def generate_launch_description():
    bringup_dir = get_package_share_directory("pb2025_nav_bringup")
    launch_dir = os.path.join(bringup_dir, "launch")

    namespace = LaunchConfiguration("namespace")
    world = LaunchConfiguration("world")
    slam = LaunchConfiguration("slam")
    map_yaml_file = LaunchConfiguration("map")
    prior_pcd_file = LaunchConfiguration("prior_pcd_file")
    use_sim_time = LaunchConfiguration("use_sim_time")
    params_file = LaunchConfiguration("params_file")
    autostart = LaunchConfiguration("autostart")
    use_composition = LaunchConfiguration("use_composition")
    use_respawn = LaunchConfiguration("use_respawn")
    use_rviz = LaunchConfiguration("use_rviz")
    use_simulator = LaunchConfiguration("use_simulator")
    rviz_config_file = LaunchConfiguration("rviz_config_file")

    declare_namespace_cmd = DeclareLaunchArgument(
        "namespace", default_value="", description="Top-level namespace"
    )

    declare_world_cmd = DeclareLaunchArgument(
        "world",
        default_value="RMUC",
        description="Gazebo world: RMUC / RMUL / RMUL2026H / RMUC2027",
    )

    declare_slam_cmd = DeclareLaunchArgument(
        "slam",
        default_value="True",
        description="True: slam_toolbox 建图；False: small_gicp 先验地图定位",
    )

    declare_map_yaml_cmd = DeclareLaunchArgument(
        "map",
        default_value=os.path.join(bringup_dir, "map", "simulation", "rmuc_2025.yaml"),
        description="先验栅格地图（仅 slam:=False 时生效）",
    )

    declare_prior_pcd_file_cmd = DeclareLaunchArgument(
        "prior_pcd_file",
        default_value=os.path.join(bringup_dir, "pcd", "simulation", "rmuc_2025.pcd"),
        description="先验点云（仅 slam:=False 时生效）",
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        "use_sim_time", default_value="True", description="Use Gazebo clock"
    )

    declare_params_file_cmd = DeclareLaunchArgument(
        "params_file",
        default_value=os.path.join(bringup_dir, "config", "simulation", "nav2_params.yaml"),
        description="Full path to the ROS2 parameters file to use for all launched nodes",
    )

    declare_autostart_cmd = DeclareLaunchArgument(
        "autostart", default_value="true", description="Automatically startup the nav2 stack"
    )

    declare_use_composition_cmd = DeclareLaunchArgument(
        "use_composition", default_value="True", description="Whether to use composed bringup"
    )

    declare_use_respawn_cmd = DeclareLaunchArgument(
        "use_respawn",
        default_value="False",
        description="Whether to respawn if a node crashes. Applied when composition is disabled.",
    )

    declare_use_rviz_cmd = DeclareLaunchArgument(
        "use_rviz", default_value="True", description="Whether to start RVIZ"
    )

    declare_use_simulator_cmd = DeclareLaunchArgument(
        "use_simulator",
        default_value="True",
        description="Whether to start pb_rm_simulation (set false if Gazebo is already running)",
    )

    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        "rviz_config_file",
        default_value=os.path.join(bringup_dir, "rviz", "nav2_default_view.rviz"),
        description="Full path to the RVIZ config file to use",
    )

    # 仿真器包是可选的：未安装/未构建时只起导航栈，不阻塞 launch 加载
    try:
        sim_launch_file = os.path.join(
            get_package_share_directory("pb_rm_simulation"), "launch", "rm_simulation.launch.py"
        )
    except Exception:
        sim_launch_file = ""

    if sim_launch_file:
        start_simulator_cmd = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(sim_launch_file),
            condition=IfCondition(use_simulator),
            launch_arguments={
                "world": world,
                "use_sim_time": use_sim_time,
            }.items(),
        )
    else:
        start_simulator_cmd = LogInfo(
            msg="[sim] 未找到 pb_rm_simulation，跳过 Gazebo 启动，只启动导航栈。"
        )

    start_bringup_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, "bringup_launch.py")),
        launch_arguments={
            "namespace": namespace,
            "slam": slam,
            "map": map_yaml_file,
            "prior_pcd_file": prior_pcd_file,
            "use_sim_time": use_sim_time,
            "params_file": params_file,
            "autostart": autostart,
            "use_composition": use_composition,
            "use_respawn": use_respawn,
        }.items(),
    )

    start_rviz_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, "rviz_launch.py")),
        condition=IfCondition(use_rviz),
        launch_arguments={
            "namespace": namespace,
            "use_sim_time": use_sim_time,
            "rviz_config": rviz_config_file,
        }.items(),
    )

    ld = LaunchDescription()

    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_world_cmd)
    ld.add_action(declare_slam_cmd)
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_prior_pcd_file_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_use_composition_cmd)
    ld.add_action(declare_use_respawn_cmd)
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_use_simulator_cmd)
    ld.add_action(declare_rviz_config_file_cmd)

    ld.add_action(start_simulator_cmd)
    ld.add_action(start_bringup_cmd)
    ld.add_action(start_rviz_cmd)

    return ld
