import sys

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
import numpy as np
import open3d as o3d


def create_pointcloud2_msg(points: np.ndarray, frame_id: str = "map") -> PointCloud2:
    num_points = points.shape[0]
    fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
    ]

    data = points.astype(np.float32).tobytes()

    header = Header(frame_id=frame_id, stamp=rclpy.clock.Clock().now().to_msg())

    return PointCloud2(
        header=header,
        height=1,
        width=num_points,
        fields=fields,
        is_bigendian=False,
        point_step=12,
        row_step=12 * num_points,
        data=data,
        is_dense=True,
    )


class PcdPublisherNode(Node):
    def __init__(self):
        super().__init__("pcd_publisher")

        self.declare_parameters(
            namespace="",
            parameters=[
                ("pcd_file", "/home/xyt/ai/pointcloud/pcd_publisher/resource/sample.pcd"),
                ("topic_name", "/pointcloud"),
                ("frame_id", "map"),
                ("publish_rate", 10.0),
            ],
        )

        self._pcd_file = (
            self.get_parameter("pcd_file").get_parameter_value().string_value
        )
        self._topic_name = (
            self.get_parameter("topic_name").get_parameter_value().string_value
        )
        self._frame_id = (
            self.get_parameter("frame_id").get_parameter_value().string_value
        )
        self._publish_rate = (
            self.get_parameter("publish_rate").get_parameter_value().double_value
        )

        self.get_logger().info(
            f"Loading PCD file: {self._pcd_file}, rate: {self._publish_rate} Hz"
        )

        try:
            pcd = o3d.io.read_point_cloud(self._pcd_file)
            self._points = np.asarray(pcd.points, dtype=np.float32)
            self.get_logger().info(f"Loaded {self._points.shape[0]} points.")
        except Exception as e:
            self.get_logger().error(f"Failed to load PCD file: {e}")
            sys.exit(1)

        self._publisher = self.create_publisher(PointCloud2, self._topic_name, 10)

        period = 1.0 / max(self._publish_rate, 0.1)
        self._timer = self.create_timer(period, self._timer_callback)

        self._seq = 0
        self.get_logger().info(
            f"Publishing PointCloud2 on [{self._topic_name}] at {self._publish_rate} Hz"
        )

    def _timer_callback(self):
        msg = create_pointcloud2_msg(self._points, frame_id=self._frame_id)
        self._publisher.publish(msg)

        stamp = self.get_clock().now().to_msg()
        sec = stamp.sec + stamp.nanosec / 1e9
        self.get_logger().debug(
            f"Published PointCloud2 #{self._seq} "
            f"({self._points.shape[0]} pts) at {sec:.3f}s"
        )
        self._seq += 1


def main(args=None):
    rclpy.init(args=args)
    node = PcdPublisherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
