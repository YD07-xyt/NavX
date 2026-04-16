DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
source $DIR/install/setup.bash  
ros2 launch standard_robot_pp_ros2 standard_robot_pp_ros2.launch.py
