# MA-sentry-2026


## env

```bash
wget http://fishros.com/install -O fishros && . fishros 
rosdepc install -r --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y
```

```bash
# 更新软件源索引
sudo apt update

sudo apt install ros-humble-serial-driver
sudo apt install ros-humble-asio-cmake-module
```
```bash
sudo apt install -y libeigen3-dev libomp-dev

git clone https://github.com/koide3/small_gicp.git
cd small_gicp
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release && make -j
sudo make install
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
./run.sh
```
使用串口
```bash
./serial.sh
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
