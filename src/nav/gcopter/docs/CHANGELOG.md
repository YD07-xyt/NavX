# 更改说明 (Changelog)

记录近期对规划/控制模块的修改。仅改动 FSM 与控制器相关模块，未触及 A*、地图、轨迹优化等其他模块。

---

## 2026-07-14

### 1. A* 参数读取与速度调参（config.hpp / fsm.hpp / astar.hpp / global_planning.yaml）

**问题**
- `set_astar_param` 原来按值传参（`AStar astar`），设置的 A* 参数作用在临时副本上被丢弃，等于没设；且漏掉 `setMaxVelocity`。
- `Config::map_size/resolution`、`FSMConfig::safe_threshold_`、`astar_param_` 各成员未初始化默认值，参数漏读时回退成垃圾值。
- 规划失败：`safe_threshold=0.9` 过大，目标点净空仅 0.951m，A* 在 0.9m 净空下找不到通道 → `A* planning failed!`。降到 0.45 恢复。
- 移动极慢：`/cmd_vel` 仅 ~0.08 m/s。诊断 `[speed-diag]` 显示 `traj_max_vel=1.47 m/s`：
  `yaw_weight=0.5` 把绕障路径的大转角膨胀进 `weighted_length`，A* 时长被算成 10.6s；
  优化器又被 `rho_energy` 压过 `rho_T`，把时长进一步拉到 15.5s。三个 3.0 的速度上限根本没触达。

**修改**
- `set_astar_param` 改为传引用 `AStar&`，并补 `setMaxVelocity`。
- 给 `map_size/resolution/safe_threshold_/astar_param_` 加合理默认值。
- `fsm.hpp` 失败分支与规划成功后加 `[diag]`/`[speed-diag]` 诊断日志。
- yaml 提速（保持 `safe_threshold` 不变）：`max_vel 3→6`、`max_acc 1.5→3`、`yaw_weight 0.5→0.2`、
  `rho_T 100→200`、`rho_energy 100→50`、`max_v 3→6`、`u_max_x/y 3→6`、`u_max_w 2→4`。
- 详见 `docs/SPEED_TUNING.md`。

---

## 2026-07-09

### 1. FSM 重规划：车辆靠近障碍物时无法规划 (`include/fsm/fsm.hpp`)

**问题**
- `AStar::originalAStarSearch` 在开始规划时判断起点是否在碰撞内：`getDistance(start) < safe_threshold_`（阈值 0.45m）。
- 当车辆实际紧贴障碍物（< 0.45m）时，起点被判为「碰撞」，A* 直接返回空路径，FSM 报 `astar_path_empty`，重规划失败。
- FSM 自身 `checkCollision` 也会因路径起点离障碍过近而判定整条路径碰撞，触发反复重试直至耗尽。

**修改**
- 新增私有方法 `getSafeStart(pos)`：当点离障碍物过近时，沿 ESDF 梯度方向（`getDistanceAndGradient`，梯度指向远离障碍物方向）外推到安全距离（`safe_threshold_`）。限制最大外推距离 1.5m，避免把起点推得太远；梯度退化时停止外推。
- 在 `plan()` 中，对起点与终点分别调用 `getSafeStart()` 后再交给 A*。A* 与碰撞检测因此可通过，规划从「安全起点」（贴近车辆、远离障碍）开始。
- 车辆仍从真实位置出发，MPC 先小幅退离障碍再沿路径跟踪，行为更安全。

### 2. MPC 跟踪中途停下 (`include/controller/traj_tracker.hpp`)

**问题**
- 滑动参考窗口仅当车辆与「窗口第一个参考点」距离 < 0.1m 时才前移。车辆越过该点后永远不会再次满足条件，窗口卡在起点附近约 1.1m 处；车辆追平该固定段后收敛、停下，表现为「跟踪中途停止」。

**修改**
- 每个控制周期改为：在整条参考轨迹上寻找**离当前车辆状态最近的点**，并以该点推进窗口（单调只前进、不后退），窗口始终跟随车辆前进，直至目标点。
- 新增 `#include <limits>`。

### 3. 提高车辆行驶速度 (`include/controller/omni_mpc.hpp`, `src/omni_mpc.cpp`)

**问题**
- 原 MPC 仅做位置跟踪，参考窗口长度仅约 1.1m，车辆被「按窗口长度配速」≈1.1 m/s，即使提高速度上限也不会更快。

**修改**
- 提高速度上限：`v_x/v_y` 4→6 m/s，`w` 2→3 rad/s。
- 新增**巡航速度项**：代价中加入 `(v_x − cruise_speed_)²`，鼓励车辆沿参考航向保持期望前进速度（默认 `cruise_speed_ = 2.0 m/s`，提供 `setCruiseSpeed()` 可调）。
- 新增参数 `cruise_gain_`：在 `solve()` 中按「到目标距离」线性衰减，进入 `stop_radius = 1.5 m` 停车区后增益归零，保证到达目标仍能平滑停下，不会冲过头。
- 权重调参：位置权重 `Q` 中航向项 1→2（跟踪更紧），控制惩罚 `R` 1→0.5（更敢给大速度）。

---

## 2026-07-13

### 1. MPC 严重偏离轨迹 (`include/controller/omni_lmpc.hpp`)

**问题**
- 当前实际启用的控制器 `LMpc`（`src/ros2_2d.cpp` 调用）的运动学模型为 `x_{k+1} = x_k + dt·u`，即把控制量 `(vx, vy, w)` 当成**世界坐标系**下的速度直接积分到位置。
- 但机器人执行 `/cmd_vel` 时 `linear.x/linear.y` 是**机体坐标系**速度（前向/横向），须经当前航向 `theta` 旋转到世界系。项目中正确的机体模型见 `src/omni_mpc.cpp:50-54`：`world = [cosθ, -sinθ; sinθ, cosθ] · [vx; vy]`。
- 两者不一致：只要车辆航向不沿世界坐标轴，MPC「以为」车辆朝世界系某方向运动，实际却朝机体方向运动，从而**严重偏离参考轨迹**。

**修改**
- 在 `LMpc` 动力学约束中引入机体→世界系的旋转，按当前航向 `theta0` 线性化：`x_{k+1} = x_k + dt·R(theta0)·[vx; vy]`，`theta_{k+1} = theta_k + dt·w`。
- 构造函数中预先为旋转项（B 块非对角元）建立稀疏模式；`slover()` 每次求解时按 `theta0 = 当前航向` 重建约束矩阵并调用 `updateLinearConstraintsMatrix`，使 MPC 预测与机器人真实运动一致。
- 新增 `#include <cmath>` 以使用 `std::cos/std::sin`。

### 2. 路径越长偏移越严重（参考游标按墙钟推进 + 速度上限过低）(`include/controller/omni_lmpc.hpp`, `src/ros2_2d.cpp`)

**问题**
- 参考游标 `t_now` 由墙钟时间 `real_elapsed` 推进（`src/ros2_2d.cpp` 原 `controller_callback`），即参考点按 1:1 真实时间沿轨迹前移。
- 但机器人未必能跟上传参考轨迹的局部速度：原 `u_max = 1.0 m/s` 远低于轨迹优化时允许的 `max_v = 3.0 m/s`（`traj_opt.hpp:48`），且曲线路段需要更大横向速度。机器人一旦落后，参考光标仍不断前移，于是它持续「追一个跑在前面、还在转弯的点」→ 反复切角 → 横向偏差随路径长度累积，越长越明显。

**修改**
- 参考游标改为**按机器人实际进度**推进：每次求解在 `t_track_` 附近窗口内搜索轨迹上离当前位姿最近的点，以其时间作为参考起点（`update_track_time()`）。这样参考始终绑定在机器人附近，落后时不会再去追前方远处、正在转弯的参考点，从根本上消除切角累积偏差。
- 新增 `reset_track()`：新轨迹下发时（`plan_omni`）将 `t_track_` 归零，避免沿用上一条轨迹的游标。
- 提高控制边界与轨迹可达速度一致：`u_min/u_max` 由 ±1.0/±0.5 提到 ±3.0（线速度）、±2.0（角速度 `w`），使 MPC 有能力跟上传参考轨迹的局部速度，减少落后与切角。
- `ros2_2d.cpp` 的 `controller_callback` 不再依赖墙钟设置 `t_now`；到目标距离 < 0.05m 的停车逻辑保留。

---

## 未改动模块
- `path_searching/astar.hpp`（A* 搜索）
- `map/grid_map.hpp`、`map/rc_esdf.*`（地图/ESDF）
- `traj_optimize/traj_opt.hpp`（轨迹优化）
- `controller/lmpc.*`、`controller/lmpc_tracker.*`（未启用的 lmpc 分支）
