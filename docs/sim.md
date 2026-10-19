# 仿真

## /NavX/src/simulator/gazebo

### 修改雷达位置

在src/simulator/gazebo/pb_rm_simulation/urdf/simulation_waking_robot.xacro中

```sh
  <!-- base_link to livox_frame -->
  <xacro:arg name="xyz" default="-0.038 0.0 0.433" />
  <xacro:arg name="rpy" default="3.14 0 0" />
```
修改这个参数

### 调整雷达发布频率

src/simulator/gazebo/livox_laser_simulation_RO2/urdf/mid360.xacro中
```sh
    <update_rate>30.0</update_rate>      <!-- 发布频率 Hz，按仿真时间算 -->
```
```sh
    <samples>20000</samples>             <!-- 插件里的这个：每帧点数 -->
    <downsample>1.0</downsample>         <!-- 每 N 条射线取 1 条 -->
````