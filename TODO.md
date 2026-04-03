# TODO

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