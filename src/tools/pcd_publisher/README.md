# run

```sh
source install/setup.sh
ros2 launch pcd_publisher pcd_publisher_launch.py pcd_file:=
```

```sh
ros2 launch pcd_publisher pcd_publisher_launch.py \
  pcd_file:=/home/xyt/ai/pointcloud/pcd_publisher/map/3d_occ.pcd \
  topic_name:=/velodyne_points \
  frame_id:=world \
  publish_rate:=20.0
```