# NavX

## env

[构建环境](docs/build.md)

## Run

[RUN](docs/run.md)
export RCUTILS_COLORIZED_OUTPUT=1
```bash
source install/setup.sh
ros2 launch bringup singlenav_launch.py
```
```bash
source install/setup.sh
ros2 launch cod_bringup multiplenav_launch.py
```

```bash
 ros2 run tf2_ros static_transform_publisher --x 0.038 --y 0.0 --z 0.433 --yaw 3.14159 --pitch  3.14159  --roll 0.0 --frame-id imu --child-frame-id base_link
```

```bash
source install/setup.bash
ros2 launch livox_ros_driver2 msg_MID360_launch.py
```

```bash
source install/setup.bash
ros2 launch serial serial.launch.py
```

```bash
source install/setup.bash
ros2 launch super_lio Livox_mid360.py
```

  // 点云 实时 true or 累积 false
  bool g_visual_dense = true;