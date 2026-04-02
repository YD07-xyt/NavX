# auto_save_map.launch.py
import os
from pathlib import Path
from launch import LaunchDescription
from launch.actions import TimerAction, ExecuteProcess

def generate_launch_description():
    ld = LaunchDescription()

    file_path = Path(__file__).resolve()

    setup_path = file_path.parent.parent.parent / "install" / "setup.sh"

    

    auto_map = file_path.parent.parent / "maps" / "auto_save"


    def create_save_command(suffix: str) -> list:
        map_path = auto_map / f"auto_map_{suffix}"
        return [
            'bash', '-c',
            'source /opt/ros/humble/setup.bash && '
            'source {setup_path} && '
            'mkdir -p {auto_map} && '
            f'ros2 run nav2_map_server map_saver_cli -f {map_path}'
        ]

    intervals = [ 30, 60, 90, 120, 150, 180, 210, 240, 270, 300]
    for t in intervals:
        action = TimerAction(
            period=float(t),
            actions=[ExecuteProcess(cmd=create_save_command("$(date +%H%M%S)"), output='screen')]
        )
        ld.add_action(action)

    return ld