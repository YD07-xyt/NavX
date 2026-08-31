#!/bin/bash
source /opt/ros/humble/setup.bash
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
source $DIR/install/setup.bash
ros2 run nav2_map_server map_saver_cli -f ./src/nav/pb2025_nav_bringup/map/reality/ma --ros-args -r map:=/map