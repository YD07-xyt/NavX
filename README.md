# MA-sentry-2026

## env

### ros2
```bash
wget http://fishros.com/install -O fishros && . fishros 
rosdepc install -r --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y
```

### small_gicp 重定位
```bash
sudo apt install -y libeigen3-dev libomp-dev

git clone https://github.com/koide3/small_gicp.git
cd small_gicp
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release && make -j
sudo make install
```

### serial
使用boost::aiso
```bash
sudo apt update
sudo apt install libboost-all-dev
```

## build
```bash
./build.sh
```
```bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

## run

### 导航

```bash
./nav.bash
```
使用串口
```bash
source install/setup.bash
ros2 launch serial serial.launch.py
```

```bash
source install/setup.bash  
ros2 launch pb2025_nav_bringup rm_navigation_reality_launch.py \
world:=rmuc_2025 \
slam:=False \
use_robot_state_pub:=True
```
### 建图

```bash
./map.sh
```

```bash
source install/setup.bash  
ros2 launch pb2025_nav_bringup rm_navigation_reality_launch.py \
slam:=True \
use_robot_state_pub:=True
```

```bash
ros2 run nav2_map_server map_saver_cli -f ./src/nav/pb2025_nav_bringup/map/reality/ma --ros-args -r map:=/map
```

#### 建图补tf

map.sh 中已补tf

/map -> /odom
```bash
ros2 run tf2_ros static_transform_publisher \
  --x 0 --y 0 --z 0 \
  --roll 0 --pitch 0 --yaw 0 \
  --frame-id odom \
  --child-frame-id base_footprint \
  --ros-args -r __ns:=/red_standard_robot1
```

/odom -> /base_footprint
```bash
ros2 run tf2_ros static_transform_publisher --x 0 --y 0 --z 0 --roll 0 --pitch 0 --yaw 0 --frame-id map --child-frame-id odom --ros-args -r __ns:=/red_standard_robot1
```

### 决策

目前集成在serial_driver中

### systemctl 自启

/service 中

```bash
#修改脚本位置 
ExecStart=bash %h/code/nav_rm_2026/pb_sentry/bt_tree.bash
```

目前只需要
nav_serial.service //串口和决策
nav.service //导航
nav_tf.service //tf map->odom

注：service/nav_bt.service已集成串口中，不需要单独启动

