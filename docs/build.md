# build NavX



### ros2 

```bash
wget http://fishros.com/install -O fishros && . fishros
```

```bash
rosdepc install -r --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y
```

### mid360 驱动

#### 1. livox SDK2

##### 安装
```bash
git clone https://github.com/Livox-SDK/Livox-SDK2.git
cd ./Livox-SDK2/
mkdir build
cd build
cmake .. && make -j
sudo make install
```

##### 删除
注意：
生成的共享库和静态库安装在“/usr/local/lib”目录中。头文件安装到“/usr/local/include”目录中。

提示：移除Livox SDK2：
```bash
sudo rm -rf /usr/local/lib/liblivox_lidar_sdk_*
sudo rm -rf /usr/local/include/livox_lidar_*
```

#### 2. livox_ros-driver2

#### clone
```bash
git clone https://github.com/Livox-SDK/livox_ros_driver2.git
```
***已集成***

####  编译
关于ROS2 Humble：
```bash
source /opt/ros/humble/setup.sh
./src/io/livox_ros_driver2/build.sh humble
```
or
```bash
./scripts/livox_build.sh
```

#### 修改雷达ip 

在src/io/livox_ros_driver2/config/MID360_config.json
中
```bash
"lidar_configs" : [
    {
      "ip" : "192.168.1.190",
      "pcl_data_type" : 1,
      "pattern_mode" : 0,
      "extrinsic_parameter" : {
        "roll": 0.0,
        "pitch": 0.0,
        "yaw": 0.0,
        "x": 0,
        "y": 0,
        "z": 0
      }
    }
  ]
```
修改雷达ip

```bash
    "host_net_info" : {
      "cmd_data_ip" : "192.168.1.5",
      "cmd_data_port": 56101,
      "push_msg_ip": "192.168.1.5",
      "push_msg_port": 56201,
      "point_data_ip": "192.168.1.5",
      "point_data_port": 56301,
      "imu_data_ip" : "192.168.1.5",
      "imu_data_port": 56401,
      "log_data_ip" : "",
      "log_data_port": 56501
    }
```
改为自己电脑的ip


### odom 

#### super_lio

```bash
sudo apt install libgoogle-glog-dev libtbb-dev
```


### io

#### serial
```bash
sudo apt install libboost-all-dev
```

### nav2
```bash
# 安装 spatio_temporal_voxel_layer
sudo apt install ros-humble-spatio-temporal-voxel-layer
```