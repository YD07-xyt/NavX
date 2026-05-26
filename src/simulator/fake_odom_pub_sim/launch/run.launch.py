from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='fake_odom_pub_sim',
            executable='fake_odom_pub_sim',
            name='fake_odom_pub_sim',
            output='screen',
            emulate_tty=True,
        )
    ])