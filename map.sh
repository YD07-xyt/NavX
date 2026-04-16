#!/bin/bash

TERMINAL="gnome-terminal"

tf_map2odom="ros2 run tf2_ros static_transform_publisher \
            --x 0 --y 0 --z 0 --roll 0 --pitch 0 --yaw 0 \
            --frame-id map \
            --child-frame-id odom \
            --ros-args -r __ns:=/red_standard_robot1"


tf_odom2base="ros2 run tf2_ros static_transform_publisher \
            --x 0 --y 0 --z 0 \
            --roll 0 --pitch 0 --yaw 0 \
            --frame-id odom \
            --child-frame-id base_footprint \
            --ros-args -r __ns:=/red_standard_robot1"


source_cmd="source install/setup.bash"


build_map="ros2 launch pb2025_nav_bringup rm_navigation_reality_launch.py \
            slam:=True \
            use_robot_state_pub:=True"


eval $source_cmd
echo "✅ ROS2环境已加载"


bash -c "$tf_map2odom" &
pid_tf1=$!
echo "✅ TF(map→odom)已启动,PID: $pid_tf1"


bash -c "$tf_odom2base" &
pid_tf2=$!
echo "✅ TF(odom→base_footprint)已启动,PID: $pid_tf2"


sleep 2


$TERMINAL --title "导航建图" -e "bash -c '$source_cmd && $build_map; read -p \"按回车关闭窗口...\"'" &
pid_build=$!
echo "✅ 建图终端已启动,PID: $pid_build"


wait $pid_tf1 $pid_tf2 $pid_build
echo "所有ROS2节点已退出,脚本结束"