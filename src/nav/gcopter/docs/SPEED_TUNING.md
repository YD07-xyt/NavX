# 速度调参指南与问题记录（global_planning）

> 适用范围：`gcopter` 全局规划节点（`global_planning_node`）。
> 配套参数文件：`config/global_planning.yaml`。
> 配套代码：`include/fsm/fsm.hpp`、`include/path_searching/astar.hpp`、
> `include/traj_optimize/traj_opt.hpp`、`include/controller/omni_lmpc.hpp`。

---

## 0. 一句话结论

机器人实际速度由**三道速度上限**共同决定，必须保持一致：

```
astar.max_vel  ==  traj.max_v  ==  lmpc.u_max_x/y
```

三者任一偏低都会卡住实际速度。参考轨迹自身的峰速则由 A* 的 `weighted_length`
（含 `yaw_weight × 总转角`）和 `max_vel / max_acc` 决定，与上面三道上限共同构成速度天花板。

**默认/保守配置下机器人只能跑 ~0.08~1.5 m/s，按本文第 3 节把三道上限提到 6.0、
降低 `yaw_weight`、并修两处代码 bug 后，可稳定提到 ~2~6 m/s（受底盘物理上限约束）。**

---

## 1. 问题现象

发布一个目标点后，规划偶尔/持续失败，或机器人移动极慢：

- 日志刷 `[error] A* planning failed!`
- 即使规划成功，`/cmd_vel_chassis` 的线速度只有 ~0.08 m/s（应达数 m/s）

---

## 2. 根因分析

### 2.1 A* 规划失败（`A* planning failed!`）

失败在 `fsm.hpp` 的 `plan()` 返回空路径，根因是 **`safe_threshold` 过大**。

- 诊断日志（在失败分支打印）显示：
  `safe_threshold=0.9`、`start_dist=1.118`、`goal_dist=0.951`
- `goal_dist=0.951` 仅比阈值 `0.9` 多 0.05m，说明目标点离障碍很近；
  整条可行通道都比 0.9m 窄，A* 在 0.9m 净空约束下**找不到一条全程离障碍 ≥0.9m 的通道**，
  开集耗尽返回空路径。
- 把 `fsm.safe_threshold` 从 0.9 降到 0.45 后恢复规划（见第 4 节当前推荐值）。

> 配套代码 bug（已修）：
> - `set_astar_param` 原来**按值传参**（`AStar astar`），设的参数作用在临时副本上被丢弃，
>   A* 参数等于没设；已改为传引用 `AStar& astar`，并补上漏掉的 `setMaxVelocity`。
> - `Config::map_size / resolution`、`FSMConfig::safe_threshold_`、`astar_param_` 各成员
>   **原来未初始化默认值**，参数漏读时回退成未初始化垃圾值；已加合理默认值。

### 2.2 机器人移动极慢（实际 ~0.08 m/s，参考轨迹峰速仅 ~1.5 m/s）

加诊断日志 `[speed-diag]` 后看到真实数据：

```
[speed-diag] astar_total_time=10.649s astar_total_len=17.191m
             traj_duration=15.493s   traj_max_vel=1.471m/s
```

两个叠加问题把速度天花板压在 1.5 m/s（远低于设置的 3.0）：

1. **`weighted_length` 被转弯严重膨胀**
   `astar.hpp::assignTrajectoryTiming` 中：
   `weighted_length = Σ(delta_s) + yaw_weight × Σ|delta_theta|`
   绕障路径总转角可达 ~30 rad，`yaw_weight=0.5` 又凭空加了 ~15m，
   等效长度从 17m 变成 ~32m。而 A* 时长 `evaluateDuration(...)` 用的就是这个膨胀后的值
   → `astar_total_time` 被算成 10.6s。

2. **优化器把时长拉得更长**（10.6s → 15.5s）
   `traj_opt.hpp::costFunction` 里时间项是 `tau_cost = rho_T × T²`（梯度为正，本应压短时长为正），
   但 `rho_energy`（平滑/低速为正，=100）压过了 `rho_T`（=100），
   优化器为了“更平滑”主动放慢 → 时长反而被拉长。

结果：`max_vel/max_v/u_max` 这三个 3.0 的上限**根本没触达**——
参考轨迹自己就只有 1.47 m/s 的峰速，机器人自然快不起来。

---

## 3. 调参方法（提速，同时保持安全）

### 3.1 三道速度上限必须一致

| 参数 | 文件位置 | 作用 | 当前推荐值 |
|---|---|---|---|
| `max_vel` | yaml:16 (astar) | A* 参考轨迹速度上限（决定 `total_time`） | **3.2** |
| `max_v` | yaml:49 (traj) | 轨迹优化硬上限（速度约束 `v ≤ max_v`） | **3.2** |
| `u_max_x / u_max_y` | yaml:68-69 (lmpc) | MPC 输出速度上限，**必须 ≥ 参考速度否则跟不上** | **3.2** |
| `u_max_w` | yaml:70 (lmpc) | 最大角速度（转弯快慢） | **4.0** |

> 三者不一致的后果（已在 CHANGELOG 记录）：`u_max` 低于参考速度 → 机器人落后 →
> 持续追一个还在前移的参考点 → 反复切角 → 横向偏差累积。
>
> 速度三上限当前统一封顶 **3.2 m/s**（用户要求 max<3.2）。需要更高速时三者统一上调，
> 受底盘/电机物理上限约束；路径越长平均越接近该上限，短路径平均偏低。

### 3.2 缩短参考轨迹时长的关键参数

| 参数 | 作用 | 当前推荐值 | 说明 |
|---|---|---|---|
| `yaw_weight` | 控制转弯在 `weighted_length` 中的权重 | **0.2**（原 0.5） | **最关键**。绕障路径转角大，调小直接砍掉 weighted_length 膨胀，A* 时长骤降 |
| `max_acc` | 加减速能力 | **3.0**（原 1.5） | 中短路径走三角速度剖面，时长 ≈ `2·√(L/max_acc)`，提速主要靠它 |
| `rho_T` | 时间惩罚（逼短时长为正） | **200**（原 100） | 压住优化器把时长拉长的倾向 |
| `rho_energy` | 平滑/低速惩罚 | **50**（原 100） | 降低“为平滑而减速”的倾向，配合提速 |

### 3.3 安全相关参数（保持，勿为提速而调小）

| 参数 | 当前值 | 说明 |
|---|---|---|
| `fsm.safe_threshold` | **0.45** | 规划期碰撞阈值。0.9 会因通道太窄导致 A* 失败；0.3 是 A* 构造默认值但余量小；0.45 为折中 |
| `traj.safe_threshold` | **0.7** | 优化期碰撞检测阈值，比规划期更严，保持 |
| `traj.rho_collision` | **1000** | “安全 vs 效率”核心权重，保持 |

### 3.4 调试用的诊断日志（已内置，无需删）

- `fsm.hpp` 失败分支打印 `[diag]`：`safe_threshold`、地图尺寸/分辨率、起点终点坐标及离障碍距离。
- `fsm.hpp` 规划成功后打印 `[speed-diag]`：`astar_total_time / astar_total_len /
  traj_duration / traj_max_vel`。
  调参时看这两行即可确认：瓶颈是“规划失败 / 参考慢 / 控制器跟不上”。

---

## 4. 当前推荐参数（config/global_planning.yaml）

目标：实际速度 **max < 3.2 m/s、平均 1.8~2.0 m/s**，且优化路径**不穿障**。

```yaml
# A* 路径搜索
max_vel: 3.2          # 速度封顶（用户要求 max<3.2）
max_acc: 3.0
yaw_weight: 0.2       # 砍掉绕障转角对 weighted_length 的膨胀

# FSM 安全（保持）
fsm:
  safe_threshold: 0.45

# TrajOpt
traj:
  rho_collision: 8000.0   # 防穿障：原 1000 太弱，平滑时样条会拱进障碍
  rho_T: 200.0
  rho_energy: 50.0
  max_v: 3.2              # 硬上限，与 max_vel / u_max 一致
  safe_threshold: 0.6     # 略高于 fsm(0.45) 留余量；过高在窄通道会无可行解
  int_K: 48               # 碰撞采样更密，避免漏检穿障点

# LMPC
lmpc:
  u_max_x: 3.2
  u_max_y: 3.2
  u_max_w: 4.0
  u_min_x: -3.2
  u_min_y: -3.2
  u_min_w: -4.0
```

> 速度三上限 `max_vel == max_v == u_max_x,y == 3.2` 必须一致。
> 若需更高速，三者统一上调即可（受底盘/电机物理上限约束）。
> 路径长度越长，平均越接近 max（长路径 avg→3.2）；短路径 avg 偏低。目标 1.8~2.0
> 对中等长度路径自然满足，极短/极长路径会偏离。

---

## 4.1 防穿障专项说明

**现象**：规划成功、能跑，但 `optimized_path`（样条）穿过障碍物。

**根因**（`traj_opt.hpp::calculateConstraintCostGrad`，碰撞项）：
- 碰撞代价是**软惩罚**，且 `cViola ≥ 0.1` 时 `penalty = cViola`（线性、梯度恒定幅度），
  权重 `rho_collision=1000` 不够强；平滑（缩短路径/降能量）的收益超过小幅穿障的惩罚，
  优化器就接受穿障。
- 碰撞采样 `int_K=32` 偏稀，样条局部穿障点可能漏检。

**修复**：
- `rho_collision` 1000 → **8000**（强惩罚，基本消除穿障）。
- `int_K` 32 → **48**（更密采样，不漏检）。
- `traj.safe_threshold` 0.7 → **0.6**（略高于 `fsm.safe_threshold=0.45` 留余量；
  **不要**设到 0.9——A* 只保证 0.45 净空，0.9 在窄通道会无可行解，反而逼轨迹贴障或失败）。
- 若仍偶发穿障：继续上调 `rho_collision`（如 15000），或确认 `fsm.safe_threshold` 没被调得过小。

---

## 5. 调参步骤（建议顺序）

1. 先确认能规划：发目标点，看是否还刷 `A* planning failed!`。
   若失败，看 `[diag]` 的 `goal_dist` 与 `safe_threshold`——`goal_dist < safe_threshold`
   就调小 `fsm.safe_threshold`（或增大 `getSafeStart` 的 `max_push`）。
2. 发目标点，看 `[speed-diag]` 的 `traj_max_vel`：
   - 若 `traj_max_vel` 远低于 `max_v` → 参考轨迹本身慢：降 `yaw_weight`、升 `max_vel/max_acc`、升 `rho_T`、降 `rho_energy`。
   - 若 `traj_max_vel` 接近 `max_v` 但机器人实际 `cmd_vel` 远低于它 → 控制器/跟踪问题：升 `lmpc.u_max`，并检查机器人是否物理限速。
3. 逐步上调速度上限，每次实车观察是否抖动/打滑/切角，触底即收。
4. `safe_threshold` 全程不动；只有确认通道极宽时可略降，反之环境变密时略升。

---

## 6. 注意事项

- `set_astar_param` 必须**传引用**（`AStar&`），否则 A* 参数设了等于没设。
- `max_vel / max_v / u_max` 三者数值必须一致，否则最松那道卡速度。
- 速度上限不可超过底盘/电机物理能力，否则轨迹快但机器人抖动、打滑、落后。
- `safe_threshold` 与速度是一对权衡：提速时不要为过障而调小安全距离。
