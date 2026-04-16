DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
source $DIR/install/setup.bash  
ros2 launch pb2025_sentry_behavior pb2025_sentry_behavior_launch.py
