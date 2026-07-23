import sys
import argparse

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import PointCloud2, PointField
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


def generate_sample_pcd(path: str, num_points: int = 1000):
    points = np.random.rand(num_points, 3).astype(np.float32) * 10.0 - 5.0
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    o3d.io.write_point_cloud(path, pcd)
    print(f"Generated sample PCD: {path} ({num_points} points)")


class PcdPublisherNode(Node):
    def __init__(self):
        super().__init__("pcd_publisher")
        self.declare_parameters(
            namespace="",
            parameters=[
                ("pcd_file", ""),
                ("topic_name", "/pointcloud"),
                ("frame_id", "map"),
                ("publish_rate", 10.0),
            ],
        )

        pcd_file = (
            self.get_parameter("pcd_file").get_parameter_value().string_value
        )
        topic_name = (
            self.get_parameter("topic_name").get_parameter_value().string_value
        )
        frame_id = (
            self.get_parameter("frame_id").get_parameter_value().string_value
        )
        publish_rate = (
            self.get_parameter("publish_rate").get_parameter_value().double_value
        )

        self.get_logger().info(
            f"Loading PCD: {pcd_file} | Topic: {topic_name} | Rate: {publish_rate} Hz"
        )
        self._pcd_file = pcd_file
        self._topic_name = topic_name
        self._frame_id = frame_id
        self._publish_rate = publish_rate

        try:
            pcd = o3d.io.read_point_cloud(pcd_file)
            self._points = np.asarray(pcd.points, dtype=np.float32)
            self.get_logger().info(f"Loaded {self._points.shape[0]} points.")
        except Exception as e:
            self.get_logger().error(f"Failed to load PCD file '{pcd_file}': {e}")
            raise

        self._publisher = self.create_publisher(PointCloud2, topic_name, 10)
        period = 1.0 / max(publish_rate, 0.1)
        self._timer = self.create_timer(period, self._timer_callback)
        self._seq = 0

    def _timer_callback(self):
        msg = create_pointcloud2_msg(self._points, frame_id=self._frame_id)
        self._publisher.publish(msg)
        self._seq += 1


def main(args=None):
    parser = argparse.ArgumentParser(description="PCD PointCloud2 Publisher")
    parser.add_argument("--pcd", default=None, help="Path to PCD file")
    parser.add_argument("--topic", default=None, help="ROS2 topic name")
    parser.add_argument("--frame-id", default=None, help="TF frame_id")
    parser.add_argument("--rate", type=float, default=None, help="Publish rate (Hz)")
    parser.add_argument("--generate-sample", type=str, default=None,
                        help="Generate a sample PCD at given path and exit")
    parsed, unknown = parser.parse_known_args(args=args)

    if parsed.generate_sample:
        generate_sample_pcd(parsed.generate_sample)
        return

    if parsed.pcd is not None or parsed.topic is not None or parsed.frame_id is not None or parsed.rate is not None:
        print(
            "Direct CLI args are deprecated. Pass parameters via ROS2 launch or --ros-args -p key:=value."
        )
        sys.exit(1)

    rclpy.init(args=unknown)
    node = PcdPublisherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
