#!/bin/bash
source /opt/ros/humble/setup.bash
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
source $DIR/install/setup.bash  
ros2 launch pb2025_nav_bringup rm_navigation_reality_launch.py \
            slam:=True \
            use_rviz:=False \
            use_robot_state_pub:=True