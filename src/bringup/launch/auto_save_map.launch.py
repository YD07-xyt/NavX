# auto_save_map.launch.py
import os
from pathlib import Path
from launch import LaunchDescription
from launch.actions import TimerAction, ExecuteProcess

def generate_launch_description():
    ld = LaunchDescription()

    file_path = Path(__file__).resolve()

    setup_path = file_path.parent.parent.parent.parent / "install" / "setup.sh"
    auto_map = file_path.parent.parent / "maps" / "auto_save"

    intervals = [30, 60, 90, 120, 150, 180, 210, 240, 270, 300]
    
    for t in intervals:
        # 构建完整的 shell 命令字符串
        cmd = (
            f'source /opt/ros/humble/setup.bash && '
            f'source {setup_path} && '
            f'mkdir -p {auto_map} && '
            f'ros2 run nav2_map_server map_saver_cli -f {auto_map}/auto_map_$(date +%H%M%S)'
        )
        
        action = TimerAction(
            period=float(t),
            actions=[ExecuteProcess(
                cmd=['bash', '-c', cmd],
                output='screen'
            )]
        )
        ld.add_action(action)

    return ld