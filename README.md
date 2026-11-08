# NavX

## env

[构建环境](docs/build.md)

## run

### mid360驱动

```sh
source install/setup.sh
ros2 launch livox_ros_driver2 msg_MID360_launch.py
```

#### 配置
launch文件中的xfer_format 为发布的点云话题类型
publish_freq为发布频率

### 定位

### super lio
```sh
source install/setup.sh
ros2 launch super_lio Livox_mid360.py
```

### z-localization

```sh
source install/setup.sh
ros2 launch z_localization_bringup  bringup.launch.py   frontend_config:="$PWD/src/location/z-localization/config/lio.yaml"   backend_config:="$PWD/src/location/z-l
ocalization/pcd/site.yaml"   mode:=mapping rviz:=true
```

### lightning-lm(不使用)
[解决编译卡死的问题](https://blog.csdn.net/qq_35102059/article/details/134083996?fromshare=blogdetail&sharetype=blogdetail&sharerId=134083996&sharerefer=PC&sharesource=2301_80216316&sharefrom=from_link)

### ma_nav

#### 运行

```sh
source install/setup.sh
ros2 launch ma_nav run.launch.py
```

建图or导航
planner.yaml中的mapping_model 更改

保存服务的话题在planner.yaml中  save_map_srv_topic save_tunnel_regions_srv_topic

#### 保存地图
```sh
ros2 service call /save_global_map std_srvs/srv/Trigger "{}"
```

#### 保存地图的隧道语义
```sh
ros2 service call /save_tunnel_regions std_srvs/srv/Trigger "{}"
```


```sh
rviz2 -d nav.rviz
```

### decision,serial

```sh
source install/setup.sh
ros2 launch ma_decision run.launch.py
```

devision中config的param.yaml 选择是否开启决策，串口是一定打开的

### tool

```sh
source install/setup.sh
ros2 launch map_edit map_edit.launch.py 
```


### 问题

1. super lio 在接收点云 奇异值 会中断程序

2. nav