#!/bin/bash
source /opt/ros/humble/setup.bash
ros2 run tf2_ros static_transform_publisher \
            --x 0.0 --y 0.0 --z 0 --roll 0 --pitch 0 --yaw 0 \
            --frame-id map \
            --child-frame-id odom \
            --ros-args -r __ns:=/red_standard_robot1