# 静止时机器人缓慢蠕动 改造记录

## User Intent

现象：不接外部速度（没有 cmd_vel）时，机器人在出生点附近会轻微、缓慢地移动。
目标：找出原因并消除静止漂移，不改变正常行驶行为。

## Scope

- `gazebo/pb_rm_simulation/urdf/simulation_waking_robot.xacro`
  - 四个轮子关节加 `<dynamics>`
  - `mecanum_controller`（libgazebo_ros_planar_move）`<update_rate>` 100 → 1000

## Out of Scope

- 不改 world（RMUC2027 地图网格、物理步长、ODE 求解器参数）。
- 不改 launch 出生点配置。
- 不改 LIO 侧配置。
- 不改轮子摩擦（`mu1/mu2` 位于 `<link>` 下、Gazebo 不识别的问题只记录不改，避免改变现有抓地力）。

## Explorer Findings

### Files inspected

- `pb_rm_simulation/urdf/simulation_waking_robot.xacro`：4 个 `continuous` 轮关节**无任何
  `<dynamics>`**（零阻尼自由滚动）；`wheel_*` 的 `<mu1>/<mu2>` 写在 `<link>` 下（URDF 非法位置，
  Gazebo 忽略，实际用默认 mu=1.0）
- `pb_rm_simulation/world/RMUC2027_world/RMUC2027_world.world`：`max_step_size=0.001`（1 kHz），
  `real_time_update_rate=1000`，重力 -9.81；地面/出生台全部来自 `rmuc_2027.stl`（单个 trimesh），
  `<max_contacts>10</max_contacts>`、`mu=100/mu2=50`
- `pb_rm_simulation/launch/rm_simulation.launch.py`：出生点 `x=3.21 y=9.6 z=0.4 yaw=0`
- `/opt/ros/humble/lib/libgazebo_ros_planar_move.so`（反汇编 `OnUpdate`）：
  每个 update 周期无条件执行 `Model::SetLinearVel(x_world, y_world, 0)` 与
  `Model::SetAngularVel(0, 0, wz)`，速度来自缓存的 `target_cmd_vel_`（未收到 cmd_vel 时为 0）。
  周期由 `<update_rate>` 决定，本工程原值 100 Hz。

### Active logic path

planar_move 插件 `OnUpdate`（100 Hz）→ 覆写模型速度 → 中间 10 ms 内由 ODE 自由积分
（重力每 10 ms 给 -0.098 m/s 的竖直速度增量，然后被下一个 tick 归零）→ 落地后接触求解
把这部分能量转为水平方向微小位移。

### 地图几何实测（用 Python 直接解析 rmuc_2027.stl，世界位姿 10.025 3.783 -1.398）

- 出生点世界坐标 (3.21, 9.6) 下方有两层：平坦地面板 world_z=0.0195（法线 0,0,-1，水平），
  以及抬高的出生台 world_z≈0.276~0.283
- 出生台在机器人四轮接地点处的高度：y=+0.13 侧 0.2760/0.2761，y=-0.13 侧 0.2833/0.2831
  → 横向 0.26 m 上差 7.3 mm ≈ **1.5° 坡度**（三角面法线 (0, 0.027, 1)）
- 即机器人实际站在一个约 1.5° 的斜面上，且轮子零阻尼自由滚动

### Risk notes

1. 轮子零阻尼 + 斜面 + 100 Hz 速度覆写间隙 = 静摩擦/接触求解残余，是"没给速度却缓慢移动"的
   主要来源；单纯改 map 摩擦抓地力不能解决滚动。
2. 若把 update_rate 提到 1000（= 物理步长），速度覆写几乎无间隙，静止时不再蠕动；
   驾驶行为不变（同一 target_cmd_vel_，只是覆写更频繁）。`publish_rate` 仍为 10，话题频率不变。
3. 轮子加阻尼不能太大：轮半径 0.06 m，1 m/s 时 ω≈16.7 rad/s；damping 0.1 会产生 ~1.7 N·m
   阻力，接近抓地力上限，会让轮子打滑。取 damping=0.01、friction=0.05（库仑，静态止动）。
4. URDF→SDF 时 Gazebo 会把 fixed 关节链 lumping，根 link 变成 `mesh`，**生成模型里没有
   `base_link` 这个 link**（反汇编显示插件不校验 `robot_base_frame` 是否存在，只用于 odom/TF，
   本工程 `publish_odom/publish_odom_tf` 都是 false，所以无影响）。此事仅记录，不改结构。

### Recommended modification boundary

只改轮关节 `<dynamics>` 与 planar_move `<update_rate>`，不动世界文件、出生点、摩擦参数。

## Modifier Changes

### Files changed

- `pb_rm_simulation/urdf/simulation_waking_robot.xacro`

### Key changes

1. 4 个轮关节各加 `<dynamics damping="0.01" friction="0.05"/>`（模拟电机/减速器阻力）。
2. `mecanum_controller`：`<update_rate>100</update_rate>` → `<update_rate>1000</update_rate>`，
   与 world 的 `max_step_size=0.001` 对齐。

### Behavior preserved

- cmd_vel 话题与 remap（`/cmd_vel_chassis`）不变；`publish_rate`、`publish_odom`、
  `publish_odom_tf`、frame 名不变。
- 轮子 link/joint 名称、几何、质量、TF 链不变。
- 出生点、世界文件、地图网格未改。

### Behavior intentionally adjusted

- 控制/速度覆写频率 100 Hz → 1000 Hz（只有计算开销变化，odom 发布频率仍 10 Hz）。
- 轮子不再是零阻尼理想轮，静止时能靠关节库仑摩擦止住蠕动。

### Notes

`<dynamics>` 已验证会被正确写进 SDF（`gz sdf -p` 输出中在 `axis/dynamics` 下出现
`damping=0.01`、`friction=0.05`）。

## Auditor Review

### Checks performed

- [x] 关键路径检查：planar_move 反汇编确认 `SetLinearVel/SetAngularVel` 在 `OnUpdate` 中无条件调用；
      轮关节 dynamics 由 URDF→SDF 正确落到 `axis/dynamics`
- [x] diff 检查：仅 1 个 xacro 文件被改，未动 world/launch/LIO 配置
- [x] grep 检查：确认改动前 4 个轮关节均无 `<dynamics>`
- [x] XML / launch / yaml 检查：`xacro` 展开成功；展开结果经 `yaml.safe_load()` 通过
      （规避 launch 的 robot_description YAML 解析坑）；`gz sdf -p` 转换成功
- [x] 用户允许范围内的测试或静态检查：**未能实机验证**——本沙箱下 gzserver 无法稳定后台运行
      （X/GLX 与模型库网络访问受限），故未做 20 s 静止漂移的量化对比
- [ ] 如需构建，已取得用户明确许可 —— 未改 C++，无需构建

### Issues found

1. 出生台本身有约 1.5° 坡度（来自地图 STL）。若提高覆写频率后仍有极缓慢漂移，
   说明残余来自斜面本身，下一步可选：
   - 把出生点挪到平坦地面板（world_z≈0.0195）附近，或
   - 在 world 里给 `<physics><ode><solver><iters>` 提高迭代次数 / 调 `mu`，
   - 或在 map 碰撞体下加一层水平平面（改动更大，本次不做）。
2. `wheel_*` 的 `<mu1>/<mu2>` 写在 `<link>` 下属于无效位置（Gazebo 忽略，实际用默认 mu=1.0）。
   修正会改变现有抓地力，本次不动，仅记录。

### Final result

NEEDS_FIX（代码改动已完成并通过静态校验，但"静止不漂移"这一验收项需要用户在实机仿真中确认；
未执行构建，因为未改 C++，且 AGENTS.md 禁止未授权构建）

### 用户验证方式

```bash
# 终端 A
ros2 launch pb_rm_simulation rm_simulation.launch.py world:=RMUC2027
# 终端 B：每 2 秒读一次模型位姿（x y z roll pitch yaw）
watch -n 2 "gz model -m robot -p"
# 或单次
gz model -m robot -p
```
