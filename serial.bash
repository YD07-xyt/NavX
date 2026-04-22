#!/bin/bash
source /opt/ros/humble/setup.bash
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
source $DIR/install/setup.bash
ros2 launch serial serial.launch.py  