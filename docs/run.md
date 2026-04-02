# run

## livox_ros_driver2

```bash
source install/setup.sh
ros2 launch livox_ros_driver2  rviz_MID360_launch.py
```

## odom

### super Lio

```bash
source install/setup.sh
ros2 launch super_lio Livox_mid360.py
```

## nav2_planner

```bash
source install/setup.sh
ros2 launch bringup multiplenav_launch.py
```

### tf
```bash
ros2 run tf2_tools view_frames
```