import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

_PACKAGE_SHARE = get_package_share_directory("pcd_publisher")
_PACKAGE_INSTALL_PREFIX = os.path.dirname(os.path.dirname(_PACKAGE_SHARE))
_DEFAULT_PCD = os.path.join(_PACKAGE_SHARE, "resource", "sample.pcd")


def generate_launch_description():
    pcd_file_arg = DeclareLaunchArgument(
        name="pcd_file",
        default_value=_DEFAULT_PCD,
        description="Absolute path to the PCD file to publish",
    )
    topic_arg = DeclareLaunchArgument(
        name="topic_name",
        default_value="/pointcloud",
        description="Topic name for the published PointCloud2",
    )
    frame_arg = DeclareLaunchArgument(
        name="frame_id",
        default_value="world",
        description="TF frame_id for the published point cloud",
    )
    rate_arg = DeclareLaunchArgument(
        name="publish_rate",
        default_value="10.0",
        description="Publish rate in Hz",
    )

    node = Node(
        package="pcd_publisher",
        executable="pcd_publisher_node",
        name="pcd_publisher",
        output="screen",
        parameters=[
            {"pcd_file": LaunchConfiguration("pcd_file")},
            {"topic_name": LaunchConfiguration("topic_name")},
            {"frame_id": LaunchConfiguration("frame_id")},
            {"publish_rate": LaunchConfiguration("publish_rate")},
        ],
    )

    return LaunchDescription([
        pcd_file_arg, topic_arg, frame_arg, rate_arg, node,
    ])
