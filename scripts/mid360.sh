#!/bin/bash
TERMINAL="gnome-terminal"

source_cmd="source install/setup.bash"

mid360="ros2 launch livox_ros_driver2 rviz_MID360_launch.py"

imu_data="ros2 topic echo livox/imu"

eval $source_cmd

bash -c "$source_cmd && $mid360" &
pid_mid360=$!
echo "✅ mid360已启动,PID: $pid_mid360"

sleep 5

$TERMINAL --title "IMU数据监听" -- bash -c "$source_cmd && $imu_data; read -p '按回车关闭窗口...'" &
pid_imu=$!
echo "✅ IMU数据监听已启动,PID: $pid_imu"
