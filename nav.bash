DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
source $DIR/install/setup.bash  
ros2 launch pb2025_nav_bringup rm_navigation_reality_launch.py \
world:=ma \
slam:=False \
use_rviz:=True \
use_composition:=True \
use_robot_state_pub:=True


