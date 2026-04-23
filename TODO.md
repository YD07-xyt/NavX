# TODO

## 4.23

## 未解决
1.launch 统一启动
2.nav2 map 靠近的障碍物会直接消除，尝试spatio_temporal_voxel_layer/SpatioTemporalVoxelLayer , pb的强度层会直接定时清除
--->地图问题直接导致导航避障效果差
3.mppi 提高速度 ## 前端路径优化？？
4.decision优化fsm 的wait逻辑 (考虑--->bt)
5.优化tf变换
6.建图导航问题
以上基本为26赛季剩下的问题 

## 4.12

## 问题解决
1.serial 通信解决
2.nav2 参数优化
3.单雷达方案 增加点云障碍物，地面切割
## 未解决
4.12.1 super_lio点云处理后生成的nav2地图质量差，历史延迟以及离群点多(怀疑是super_lio点云累计发布？还是处理的问题？)

## 4.4

## 问题解决

nav2-dev
1.st_planner 融合进 nav2 插件

## 未解决
4.4.1 st_planner 规划太久 无法高频重规划 优先优化 路径规划 A* 

## 4.3

## 问题解决
1.x_planner jps
2.x_planner minco
3.x_planner test rmuc2026

## 未解决
4.3.1 x_planner minco轨迹振荡，
4.3.2 x_planner minco未+esdf梯度避障
4.3.3 x_planner mpc 控制
4.3.4 x_planner map(rog_map的接口)

## 4.2

增加nav2_planner(<---- cod_nav)
解决livox_ros_diver2 一起编译 (colcon 指定humble版本)

## 4.1 问题解决

4.1.1.serial 重启 暂时解决 待优化

4.1.2.path_search a*融合 rog_map  改为super_planner原生规划器

4.1.3.rog_map esdf 显示问题 解决  参数设置问题

4.1.6.planner 后端优化 ---
                    |-------- super_planner
4.1.7.路径跟随 mpc--------

## 未解决
4.1.5.decision rmuc 的决策  暂时准备所用24年的 
4.2.1 串口增加ros2 部分

4.2.2 测试少， 参数未调

## 4.1

### 未解决
4.1.1.serial 重启
4.1.2.path_search a*融合 rog_map
4.1.3.rog_map esdf 显示问题
4.1.4.rmuc 全局地图处理 opencv
4.1.5.decision rmuc 的决策
4.1.6.planner 后端优化
4.1.7.路径跟随 mpc
4.1.8.rog map测试少，对于地图动态障碍物的处理 以及地图质量(处理细微障碍物， 考虑上实车看效果)