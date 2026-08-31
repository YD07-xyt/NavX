# auto_save_map.launch.py
import os
from launch import LaunchDescription
from launch.actions import TimerAction, ExecuteProcess, LogInfo
from launch_ros.actions import Node
from launch.substitutions import PythonExpression
import time

def generate_launch_description():
    ld = LaunchDescription()
    
    # 创建一个检查节点，确保地图服务就绪后再开始保存
    readiness_check_script = f"""#!/bin/bash
    source /opt/ros/humble/setup.sh
    source /home/ma/nav/NavX/install/setup.sh
    
    echo "Waiting for map_server service to be ready..."
    
    # 等待最多30秒直到服务可用
    timeout=30
    start_time=$(date +%s)
    while [ $(($(date +%s) - start_time)) -lt $timeout ]; do
        if ros2 service list 2>/dev/null | grep -q "/map_saver/save_map"; then
            echo "Map saver service is ready!"
            exit 0
        fi
        sleep 1
    done
    
    echo "Warning: Map saver service not found after {timeout} seconds"
    exit 0  # 继续执行，不阻塞
    """
    
    # 保存检查脚本
    script_path = '/tmp/check_map_service.sh'
    with open(script_path, 'w') as f:
        f.write(readiness_check_script)
    os.chmod(script_path, 0o755)
    
    # 先执行服务检查
    service_check = ExecuteProcess(
        cmd=[script_path],
        output='screen',
        shell=True
    )
    ld.add_action(service_check)
    
    # 等待5秒确保初始化完成
    wait_for_init = TimerAction(
        period=5.0,
        actions=[
            LogInfo(msg="Starting auto map save tasks...")
        ]
    )
    ld.add_action(wait_for_init)
    
    def create_save_command(suffix: str) -> list:
        return [
            'bash', '-c',
            'source /opt/ros/humble/setup.sh && '
            'source /home/ma/nav/NavX/install/setup.sh && '
            'mkdir -p /home/ma/nav/NavX/src/nav/pb2025_nav_bringup/map/auto_save && '
            f'ros2 run nav2_map_server map_saver_cli -f /home/ma/nav/NavX/src/nav/pb2025_nav_bringup/map/auto_save/auto_map_{suffix}'
        ]

    intervals = [30, 60, 90, 120, 150, 180, 210, 240, 270, 300]
    
    for t in intervals:
        # 从launch启动开始计算时间，而不是从当前时间
        action = TimerAction(
            period=float(t + 5),  # 所有保存都在5秒延迟后开始
            actions=[ExecuteProcess(cmd=create_save_command("$(date +%H%M%S)"), output='screen')]
        )
        ld.add_action(action)

    return ld