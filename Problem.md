# Run The Project

## build

```bash
git clone https://github.com/YD07-xyt/pb_sentry.git
git clone https://github.com/SMBU-PolarBear-Robotics-Team/livox_ros_driver2.git
```
运行
```bash
chmod +x build.sh
./build.sh 
```

## 配置雷达

在pb2025_nav_bringup/config下  
[配置文件位置](pb2025_nav_bringup/config/reality/mid360_user_config.json)  
reality/mid360_user_config.json中 



***"host_net_info"下的ip改为你的ip***  
***"lidar_configs"下的ip改为雷达的ip***



注意：ivox_ros_driver2中的config修改后无效(没用到)

你的ip和雷达的ip 只有最后的一部分不一样(必须不一样)  
eg:   

正确：192.168.1.XX 和  192.168.1.XX  
   
错误：192.168.2.XX 和192.168.1.XX 


## 修改雷达位置

参考：[pb实车部署指南](https://github.com/SMBU-PolarBear-Robotics-Team/pb2025_sentry_nav/wiki/%E5%AE%9E%E8%BD%A6%E9%83%A8%E7%BD%B2%E6%8C%87%E5%8D%97#11-%E4%BF%AE%E6%94%B9%E4%BC%A0%E6%84%9F%E5%99%A8%E4%BD%8D%E7%BD%AE)


### 1. 修改gravity， gravity_init

#### 如何得到雷达当前gravity

```bash
chmod +x mid360.sh
./mid360.sh
```
查看imu的线加速度(单位：g(一个重力加速度))  
转化为单位：m/s2  

**查看到的乘以9.81**  
**注意重力方向加负号**  
填入[配置文件位置](src/pb2025_nav_bringup/config/reality/nav2_params.yaml)的 mapping:下的gravity， gravity_init处

注意： point_lio中的config修改后无效(没用到)

### 2. 修改雷达pose

[配置文件位置](src/pb2025_robot_description/resource/xmacro/pb2025_sentry_robot.sdf.xmacro)

修改第25行
```bash
<xmacro_block name="livox" prefix="front_" parent="gimbal_yaw" pose="-0.0496 0.136 0.435 -${pi/4} 0 0" update_rate="20" samples="1875"/>
```

其中只修改pose的部分  
参数意义：  
x,y,z,roll(绕x轴),pitch(绕y轴),yaw(绕z轴)  

roll ,pitch,yaw 都为0时雷达正向上

## run 
 
都为实车

### 建图
```bash
chmod +x map.sh
./map.sh
```

#### 建图补tf(map.sh自动补)
```bash
ros2 run tf2_ros static_transform_publisher \
  --x 0 --y 0 --z 0 \
  --roll 0 --pitch 0 --yaw 0 \
  --frame-id odom \
  --child-frame-id base_footprint \
  --ros-args -r __ns:=/red_standard_robot1
```

```bash
ros2 run tf2_ros static_transform_publisher --x 0 --y 0 --z 0 --roll 0 --pitch 0 --yaw 0 --frame-id map --child-frame-id odom --ros-args -r __ns:=/red_standard_robot1
```

#### 看tf
```bash
ros2 run rqt_tf_tree rqt_tf_tree --ros-args -r /tf:=tf -r /tf_static:=tf_static -r __ns:=/red_standard_robot1
```
```bash
ros2 run tf2_tools view_frames
```

### 导航
```bash
chmod +x run.sh
./run.sh
```
### 串口

```bash
sudo ufw disable
```


https://docker.1ms.run 
https://docker.m.daocloud.io/ 
https://docker.1panel.live/ https://hub.rat.dev/ https://docker.1ms.run/ https://docker.xuanyuan.me/ https://dislabaiot.xyz/ https://doublezonline.cloud/ https://xdark.top/

