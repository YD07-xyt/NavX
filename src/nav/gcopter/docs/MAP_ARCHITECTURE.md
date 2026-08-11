# ROG-Map 地图模块架构文档

> 基于 [ROG-Map](https://github.com/hku-mars/ROG-Map) (Yunfan REN, MaRS Lab, HKU)，gcopter 集成适配版本。

---

## 目录

1. [整体架构](#1-整体架构)
2. [类继承体系](#2-类继承体系)
3. [各类详解](#3-各类详解)
   - [SlidingMap — 滑动窗口基类](#31-slidingmap--滑动窗口基类)
   - [CounterMap — 亚格子计数层](#32-countermap--亚格子计数层)
   - [InfMap — 膨胀地图](#33-infmap--膨胀地图)
   - [ProbMap — 概率占据地图](#34-probmap--概率占据地图)
   - [ROGMap — 顶层地图](#35-rogmap--顶层地图)
   - [ESDFMap — 欧几里得符号距离场](#36-esdfmap--欧几里得符号距离场)
   - [FreeCntMap — 前沿检测](#37-freecntmap--前沿检测)
   - [RayCaster — DDA 射线投射](#38-raycaster--dda-射线投射)
   - [Config — 参数配置](#39-config--参数配置)
   - [MaMap — 应用层封装](#310-mamap--应用层封装)
   - [GridMap — 2D 占据栅格](#311-gridmap--2d-占据栅格)
   - [TerrainAnalyzer — 地形分析](#312-terrainanalyzer--地形分析)
4. [数据流](#4-数据流)
5. [关键算法](#5-关键算法)
6. [性能分析](#6-性能分析)
7. [已知问题与改进点](#7-已知问题与改进点)

---

## 1. 整体架构

```
                        ┌──────────────────────┐
                        │       MaMap          │  ← 应用层封装
                        │ (owns ROGMap,        │
                        │  GridMap,            │
                        │  TerrainAnalyzer)    │
                        └──────┬───────────────┘
                               │
          ┌────────────────────┼────────────────────┐
          ▼                    ▼                    ▼
   ┌──────────────┐   ┌──────────────┐   ┌────────────────┐
   │   ROGMap     │   │   GridMap    │   │ TerrainAnalyzer│
   │  (核心3D地图) │   │  (2D ESDF)  │   │ (通过性分析)    │
   └──────┬───────┘   └──────────────┘   └────────────────┘
          │
     ┌────▼────┐         ┌──────────────┐         ┌──────────────┐
     │ ProbMap  │───────▶│   InfMap     │         │   ESDFMap    │
     │(概率融合)│         │  (膨胀地图)   │         │  (ESDF距离场) │
     └────┬─────┘         └──────┬───────┘         └──────┬───────┘
          │                      │                        │
     ┌────▼─────┐         ┌──────▼───────┐         ┌──────▼───────┐
     │SlidingMap│         │  CounterMap  │         │  CounterMap  │
     │(滑动窗口)│         │ (亚格子计数)  │         │ (亚格子计数)  │
     └──────────┘         └──────┬───────┘         └──────────────┘
                                 │
                          ┌──────▼───────┐
                          │  SlidingMap  │
                          │  (滑动窗口)   │
                          └──────────────┘
```

**核心思想**：3D 概率占据地图 + 膨胀层 + ESDF 距离场，以机器人当前位置为中心的滑动窗口实现无限范围建图。使用亚格子计数器（CounterMap）实现多分辨率膨胀，避免对每个精细格子维护膨胀状态。

---

## 2. 类继承体系

```
SlidingMap                       ← 滑动窗口 + 坐标变换基类
├── CounterMap : SlidingMap      ← 亚格子计数器（多分辨率基础）
│   ├── InfMap : CounterMap      ← 膨胀（inflation）计数
│   └── ESDFMap : CounterMap     ← 欧几里得距离场
├── FreeCntMap : SlidingMap      ← 前沿检测（应继承 CounterMap，见 TODO）
└── ProbMap : SlidingMap         ← 概率融合 + 射线投射
    └── ROGMap : ProbMap         ← 顶层地图（查询接口）

独立类：
├── ma_map::MaMap                ← 应用层封装
├── grid_map::GridMap            ← 2D ESDF 栅格地图
├── Terrain::TerrainAnalyzer     ← 地形可通过性分析
├── rog_map::raycaster::RayCaster ← DDA 3D 射线投射
└── rog_map::Config              ← 参数配置
```

**继承深度**：最深的 `ESDFMap` / `InfMap` 链为 4 层（`SlidingMap` → `CounterMap` → `ESDFMap` / `InfMap`）。

---

## 3. 各类详解

### 3.1 SlidingMap — 滑动窗口基类

**文件**: `include/map/3d_occ_map/sliding_map.h`, `src/map/sliding_map.cpp`

**职责**：
- 以机器人为中心的 3D 环形缓冲区滑动窗口
- 全局坐标 ↔ 局部索引 ↔ 线性哈希的全套变换
- 地图滑动触发子类 cell 清零（虚函数 `resetCell` / `resetLocalMap`）

**坐标系**（`ORIGIN_AT_CORNER` 模式）：
```
原点在格子角点：
  bd_min     origin          bd_max
    |           |               |
    v           V               V
    -2    -1    0       1       2
|------|------|------|------|------|
-2  -1.5  -1  -0.5  0   0.5  1   1.5
```

**关键成员**：
| 成员 | 类型 | 说明 |
|------|------|------|
| `sc_` | `SlidingConfig` | resolution, half_map_size_i, map_size_i, map_vox_num 等 |
| `local_map_origin_d_/i_` | `Vec3f/Vec3i` | 当前窗口原点（世界坐标/全局索引） |
| `local_map_bound_min_/max_` | `Vec3f/Vec3i` | 窗口边界 |

**关键方法**：

| 方法 | 复杂度 | 说明 |
|------|--------|------|
| `initSlidingMap()` | O(1) | 设置元数据，计算 `map_size_i = 2*half+1`, `map_vox_num = prod` |
| `mapSliding(odom)` | O(shift_num × 二维) | 计算 shift → 清除溢出区域 → 更新原点 |
| `getLocalIndexHash(id_in)` | O(1) | `hash = x*Y*Z + y*Z + z`，行优先展平 |
| `posToGlobalIndex(pos, id)` | O(1) | `id = floor(pos / res)`（CORNER 模式） |
| `globalIndexToPos(id_g, pos)` | O(1) | `pos = (id_g + 0.5) * res` |
| `insideLocalMap(pos/id_g)` | O(1) | `|id_g - origin| ≤ half` |
| `clearMemoryOutOfMap(clear_id, i)` | O(\|clear_id\| × dim_j × dim_k) | 清除滑动越界区域，逐格调 `resetCell` |

**算法细节 — mapSliding**：
```
给定 odom：
1. posToGlobalIndex(odom) → new_origin_i
2. shift_num = new_origin_i - local_map_origin_i_
3. 若 |shift| > map_size：resetLocalMap() → 更新 origin → return（大范围跳跃全清）
4. 对每个轴：
   - 计算需要清除的环形缓冲区 slice（min_id_l 的偏移范围）
   - 调 clearMemoryOutOfMap 清除
5. updateLocalMapOriginAndBound
```

**resetCell 调用链**（从 SlidingMap → 子类）：
```
clearMemoryOutOfMap → resetCell(hash_id) [虚函数]
  CounterMap::resetCell → resetOneCell(hash_id) [纯虚]
    InfMap::resetOneCell → 清零 occ_inflate_cnt, unk_inflate_cnt
```

---

### 3.2 CounterMap — 亚格子计数层

**文件**: `include/map/3d_occ_map/counter_map.h`, `src/map/counter_map.cpp`

**职责**：实现多分辨率占用/未知计数。一个 CounterMap cell 对应若干 ProbMap cell。

**核心数据结构**：
```
md_.occupied_cnt[hash]  ← 此 cell 被占据的亚格子数量
md_.unknown_cnt[hash]   ← 此 cell 未知的亚格子数量
md_.sub_grid_num        ← 一个 counter cell 覆盖的 prob cell 数量
md_.unk_thresh          ← 未知判定阈值（unk_cnt >= unk_thresh → UNKNOWN）
```

**状态判定**：
```
occupied_cnt > 0          → OCCUPIED
unknown_cnt >= unk_thresh → UNKNOWN
否则                       → KNOWN_FREE
```

**关键方法**：

| 方法 | 复杂度 | 说明 |
|------|--------|------|
| `initCounterMap()` | O(1) | 计算分辨率比例 → 分配计数器 vector |
| `updateGridCounter(pos, from, to)` | O(1) + O(N_neighbor) 若翻转 | 增减计数器 → 若类型翻转，调 `triggerJumpingEdge` |
| `getGridType(hash_id)` | O(1) | 根据计数器判定状态 |
| `resetCell(hash_id)` | O(1) | 清零计数器 + `resetOneCell` |

**分辨率计算**（在 `initCounterMap` 中）：
```
inflation_ratio = round(counter_resolution / prob_resolution)
counter_map_resolution = prob_resolution * inflation_ratio
half_counter_map_size_i = half_prob_map_size_d / counter_resolution + (inflation_step + 1)
```

---

### 3.3 InfMap — 膨胀地图

**文件**: `include/map/3d_occ_map/inf_map.h`, `src/map/inf_map.cpp`

**职责**：在 CounterMap 基础上增加膨胀计数。当 ProbMap 中一个 cell 状态翻转时，其 spherical neighbor 的膨胀计数器 ±1。

**核心数据结构**：
```
imd_.occ_inflate_cnt[hash]   ← 膨胀占用计数 (>0 → inflated occupied)
imd_.unk_inflate_cnt[hash]   ← 膨胀未知计数
imd_.occ_neighbor_num         ← 膨胀邻居数量
```

**关键方法**：

| 方法 | 复杂度 | 说明 |
|------|--------|------|
| `updateInflation(id_g, is_hit)` | O(N_neighbor) | 遍历 spherical neighbor 增减膨胀计数 |
| `updateUnkInflation(id_g, is_add)` | O(N_neighbor) | 同上，针对未知膨胀 |
| `triggerJumpingEdge(id_g, from, to)` | O(N_neighbor) | 状态翻转时调 updateInflation |
| `isOccupiedInflate(pos)` | O(1) | `occ_inflate_cnt[hash] > 0 \|\| z 越界` |
| `boxSearch(box, type, out)` | O(box_size) | 三重循环全遍历（无早期终止） |

**热点路径**：`triggerJumpingEdge` → `updateInflation` → 对每个 spherical neighbor 做 hash lookup + ±1 —— 每个 cell 状态翻转产生 N_neighbor 次离散内存访问。

**inf_spherical_neighbor**：
```cpp
// 在 Config 构造函数中预计算
for (dx=-step; dx<=step; dx++)
 for (dy=-step; dy<=step; dy++)
  for (dz=-step; dz<=step; dz++)
   if (dx²+dy²+dz² <= step²)
     inf_spherical_neighbor.push_back(Vec3i(dx,dy,dz));
// 按距离排序
```

---

### 3.4 ProbMap — 概率占据地图

**文件**: `include/map/3d_occ_map/prob_map.h`, `src/map/prob_map.cpp`

**职责**：核心概率占用地图层。接收点云 → 射线投射 → 批量更新 log-odds 概率。持有 InfMap、FreeCntMap、ESDFMap 子地图。

**核心数据结构**：
```
occupancy_buffer_[sc_.map_vox_num]    ← log-odds 概率（double），每个 cell 的占据概率
raycast_data_:
  raycaster         ← DDA raycaster 实例
  update_cache_id_g ← 批量更新队列 (queue<Vec3i>)
  operation_cnt[]   ← 本批次每个 cell 的 hit+miss 总次数
  hit_cnt[]         ← 本批次每个 cell 的 hit 次数
  batch_update_counter ← 帧计数，达到 batch_update_size 时执行概率融合
```

**概率更新（log-odds）**：
```
l_hit  = log(p_hit / (1-p_hit))
l_miss = log(p_miss / (1-p_miss))
cell_prob += hit_num × l_hit   // hit 更新
cell_prob += miss_num × l_miss // miss 更新
clamp(cell_prob, l_min, l_max)
```

**状态判定**：
```
prob < l_free  → KNOWN_FREE
prob >= l_occ  → OCCUPIED
否则           → UNKNOWN
```

**关键方法**：

| 方法 | 复杂度 | 说明 |
|------|--------|------|
| `initProbMap()` | O(1) | 调 initSlidingMap + 创建子 map + 分配 buffer |
| `updateProbMap(cloud, pose)` | O(C + R) | 主更新入口（见数据流章节） |
| `raycastProcess(cloud, odom)` | O(\|cloud\| × ray_steps) | 对每个点做 DDA 射线投射 |
| `hitPointUpdate(pos, hash, num)` | O(1) + O(N) 若翻转 | log-odds 更新 + 触发子地图 |
| `missPointUpdate(pos, hash, num)` | O(1) + O(N) 若翻转 | 同上 |
| `probabilisticMapFromCache()` | O(\|unique_cells\|) | 批量消费更新队列 |
| `insertUpdateCandidate(id_g, is_hit)` | O(1) | 进队 + counter++ |
| `boxSearch(box, type, out)` | O(box_size) | 三重循环遍历 |
| `boxSearchInflate(box, type, out)` | O(box_size) | 委托 `inf_map_->boxSearch` |
| `resetLocalMap()` | O(map_vox_num) | `std::fill` 全 buffer |

---

### 3.5 ROGMap — 顶层地图

**文件**: `include/map/3d_occ_map/rog_map.h`, `src/map/rog_map.cpp`

**职责**：ROG-Map 的最外层用户接口。在 ProbMap 之上提供射线查询和最近邻搜索。

**关键方法**：

| 方法 | 复杂度 | 说明 |
|------|--------|------|
| `init()` | O(1) | 链式初始化 ProbMap → 可选 PCD 加载 |
| `isLineFree(start, end)` | O(ray_steps) | 单线无障碍检测 |
| `isLineFree(start, end, neighbor_list)` | O(ray_steps × \|neighbor\|) | 带 neighbor 膨胀 |
| `isLineFree(start, end, free_goal)` | O(ray_steps) | 返回最后一个自由点 |
| `getNearestCellIs(type, start, pt, max_dis)` | O(\|spherical_neighbor\|) | 螺旋搜索最近匹配 cell |
| `getNearestCellNot(type, start, pt, max_dis)` | O(\|spherical_neighbor\|) | 螺旋搜索最近不匹配 cell |
| `updateRobotState(pose)` | O(1) | 更新 robot_state_ + updateLocalBox |
| `updateMap(cloud, pose)` | O(C+R) | 外部更新入口（ros_callback_en=false 时使用） |

---

### 3.6 ESDFMap — 欧几里得符号距离场

**文件**: `include/map/3d_occ_map/esdf_map.h`, `src/map/esdf_map.cpp`

**职责**：基于 CounterMap 的占据状态，使用 Felzenszwalb-Huttenlocher 距离变换算法构建 3D ESDF。

**核心算法**：
```
updateESDF3D(cur_odom):
  1. 计算局部更新 box（以 odom 为中心的 local_update_box）
  2. 正距离（到占据）: fillESDF(z-line) → x-line → y-line
  3. 负距离（到自由）: 同上，合并至 distance_buffer
```

**`fillESDF`（1D 抛物线包络）**：
- O(N) 时间，O(N) 辅助栈
- VLA 分配：`int v[map_size]`, `double z[map_size+1]`

**查询**：
- `getDistance(pos)` → 从 distance_buffer 采样
- `evaluateFirstGrad(pos)` → 三线性插值求梯度
- `evaluateSecondGrad(pos)` → 三线性插值求二阶梯度

---

### 3.7 FreeCntMap — 前沿检测

**文件**: `include/map/3d_occ_map/free_cnt_map.h`

**职责**：统计每个 cell 的 26 邻居中 known-free 的数量，用于判定是否为 frontier（自由但有未知邻居）。

**已知问题**：有 TODO 注释 "inherit from counter map"，但当前直接继承 `SlidingMap`，与 CounterMap 分支平行，存在代码重复。

---

### 3.8 RayCaster — DDA 射线投射

**文件**: `include/map/3d_occ_map/raycaster.h`, `src/map/raycaster.cpp`

**职责**：3D DDA (Digital Differential Analyzer) 算法，沿射线步进遍历所有相交的栅格。

**算法**（`setInput` + `step`）：
```
setInput(start, end):
  1. 计算 start/end 的 global index
  2. 计算各轴方向 expand_dir (+1/-1)
  3. 计算 t_when_step（每步 t 增量）和 t_to_bound（到下一边界的初始 t）

step():
  1. 输出当前 cell 的中心坐标
  2. 若到达 end → return false
  3. 比较 t_to_bound_x/y/z，走最小值对应轴
  4. 该轴 index += expand_dir, t_to_bound += t_when_step
  5. return true
```

复杂度：O(ray_steps)，每步 O(1)。ray_steps ≈ ray_length / resolution。

---

### 3.9 Config — 参数配置

**文件**: `include/map/3d_occ_map/config.hpp`

**参数分类**：

| 类别 | 参数 | 默认值 | 说明 |
|------|------|--------|------|
| 地图尺寸 | `resolution` | 0.1 | ProbMap 分辨率 (m) |
| | `inflation_resolution` | 0.1 | InfMap 分辨率 (m)，≥ resolution |
| | `map_size` | [10,10,0] | 地图长宽高 (m) |
| | `inflation_step` | 1 | 膨胀步长（球面半径） |
| 滑动窗口 | `map_sliding.enable` | true | 是否启用滑动 |
| | `map_sliding.threshold` | -1 | 滑动触发距离阈值 |
| | `fix_map_origin` | [0,0,0] | 固定原点（关闭滑动时使用） |
| 射线投射 | `raycasting.enable` | true | 是否启用 DDA |
| | `raycasting.ray_range` | [0.3,10] | 射线最小/最大距离 |
| | `raycasting.batch_update_size` | 1 | 批处理帧数 |
| | `raycasting.local_update_box` | [999,999,999] | 局部更新范围 |
| 概率参数 | `raycasting.p_hit/miss/min/max/occ/free` | — | log-odds 概率参数 |
| | `raycasting.unk_thresh` | 0.7 | 未知判定阈值 |
| ROS | `ros_callback.enable` | false | 是否启用 ROS 回调 |
| | `ros_callback.cloud_topic` | "/lio/cloud_world" | 点云话题 |
| | `ros_callback.odom_topic` | "/lio/odom" | 里程计话题 |
| | `ros_callback.odom_timeout` | 0.05 | 里程计超时 (s) |
| 可视化 | `visualization.enable` | true | 是否启用可视化 |
| | `visualization.range` | [0,0,0] | 可视化范围 (m) |
| | `visualization.time_rate` | 0 | 发布频率上限 (Hz) |
| 其他 | `virtual_ceil_height` | -0.1 | 虚拟天花板 (m) |
| | `virtual_ground_height` | -0.1 | 虚拟地面 (m) |
| | `esdf.enable` | false | 是否启用 ESDF |
| | `intensity_thresh` | -1 | 点云强度滤波阈值 |

**`resetMapSize()` 计算链**：
```
1. inflation_ratio = ceil(inflation_resolution / resolution)
2. inflation_resolution = resolution × inflation_ratio
3. inf_half_map_size_i = (half_map_size_d / inflation_resolution) + (max_step+1)
4. half_map_size_i = (inf_half_map_size_i - (max_step+1)) × inflation_ratio
5. map_size_d = (half_map_size_i × 2 + 1) × resolution  ← 重新对齐
```

---

### 3.10 MaMap — 应用层封装

**文件**: `include/map/ma_map.hpp`

**职责**：将 ROGMap（3D 概率地图）、GridMap（2D ESDF 地图）和 TerrainAnalyzer（地形分析）组合在一起，作为 gcopter 规划器的主数据源。

**成员**：
```
rog_map_ptr_   ← shared_ptr<ROGMap>
grid_map_ptr_  ← shared_ptr<GridMap>
terrain_analyzer_ ← TerrainAnalyzer
cfg_           ← rog_map::Config
cloud_ / pc_pose_ ← 缓存最新云和位姿
occupancy_2d   ← 2D 占据矩阵 (RowMatrixXi)
```

**关键方法**：

| 方法 | 说明 |
|------|------|
| `update_odom(pose)` | 转发给 `rog_map_ptr_->updateRobotState(pose)` |
| `update_cloud(cloud)` | 缓存点云+位姿（含 odom 有效性/timeout 检查）→ `map_empty_ = false` |
| `update_map()` | 消费缓存 → `updateProbMap` → `update_terrain` → 填充 `GridMap` |
| `update_terrain()` | boxSearch(full viz range) → terrain analyzer → 标记 GridMap |

**线程安全**：`updete_lock`（互斥锁，保护 cloud_/pc_pose_ 交换），但注释拼写错误 `updete`。

**已知问题**：
- `update_terrain()` 使用 `visualization_range`(±50m) 做 boxSearch，而 GridMap 只有 2m×2m，大量点被丢弃。
- `MaMap::cfg_` 和 `ROGMap::cfg_` 是两个独立拷贝，需显式 `setConfig` 同步。

---

### 3.11 GridMap — 2D 占据栅格

**文件**: `include/map/grid_map.hpp`

**职责**：2D 占据栅格 + ESDF 距离场。用于 A* 路径规划。

**ESDF 算法**：Felzenszwalb-Huttenlocher 两遍距离变换。

**查询**：`getDistance(pos)` — 双线性插值采样 ESDF。

**当前实例化**：`grid_map_ptr_->init(41, 41, 0.05)` → 2.05m × 2.05m，机器人周围 ±1m。

---

### 3.12 TerrainAnalyzer — 地形分析

**文件**: `include/map/terrain_analysis.hpp`, `src/map/terrain_analysis.cpp`

**职责**：从 3D 占据点云提取可通过性信息。5 阶段分析：

```
1. setupGrid()      ← 以机器人为中心的 2D 网格
2. 点云分配          ← 每点 worldToGrid，分配到对应 cell
3. 提取高度          ← 每 cell 的 min_z / max_z
4. 中值滤波          ← kernel_size × kernel_size 窗口
5. 梯度+间隙分析     ← 8 邻域梯度 + 高度间隙 > robot_height → 障碍
```

**瓶颈**：中值滤波（`kernel_size=5` 时每 cell 25 点排序）和 per-cell 点云分配（点数不均匀时可能有长尾）。

---

## 4. 数据流

### 4.1 主更新路径

```
10Hz 点云 → map_callback (GlobalPlanner2d)
  ├─ update_cloud(pc)
  │   ├─ 检查 odom 有效性 (rcv + timeout)
  │   ├─ updete_lock → cloud_ = pc, pc_pose_ = (robot_state.p, q)
  │   └─ map_empty_ = false
  │
  └─ update_map()
      ├─ updete_lock → 取出 temp_pc, temp_pose → 重置 counter
      │
      ├─ updateProbMap(temp_pc, temp_pose)   ← 主概率更新
      │   ├─ 滑动检查: 若需滑动 → slideAllMap
      │   ├─ updateLocalBox
      │   ├─ raycastProcess(cloud, odom)      ← DDA 射线投射
      │   │   ├─ 遍历点云 → 裁剪 (ceil/ground, range, update_box)
      │   │   ├─ insertUpdateCandidate(true)   ← hit
      │   │   └─ RayCaster::step → insertUpdateCandidate(false) ← free
      │   ├─ batch_update_counter++ → 达到 batch_update_size:
      │   │   └─ probabilisticMapFromCache
      │   │       ├─ hitPointUpdate: prob += l_hit → updateGridCounter → triggerJumpingEdge
      │   │       └─ missPointUpdate: prob += l_miss → 同上
      │   └─ ESDF update (若启用)
      │
      ├─ update_terrain()
      │   ├─ boxSearchInflate(full_viz_range, OCCUPIED) → inf_occ_map
      │   ├─ boxSearch(full_viz_range, OCCUPIED) → real_occ_map
      │   └─ terrain_analyzer.analyze(robot_state, real_occ_map)
      │       └─ 输出 terrain_map_ (障碍物位置列表)
      │
      └─ 填充 GridMap: terrain_map_ → occupancy_2d → setMap
          └─ GridMap::updateESDF → 2D 距离场
```

### 4.2 可视化发布路径

```
100Hz pub_viz_timer_ (实际受 rate limit 限制)
  → pub_callback
    → map_viz_callback
      ├─ boxSearch(viz_range, UNKNOWN)    → /ma_nav/map/unk
      ├─ boxSearchInflate(viz_range, UNKNOWN) → /ma_nav/map/inf_unk
      ├─ boxSearch(viz_range, OCCUPIED)   → /ma_nav/map/occ      ← 主话题
      ├─ boxSearchInflate(viz_range, OCCUPIED) → /ma_nav/map/inf_occ
      ├─ boxSearch(viz_range, FRONTIER)   → /ma_nav/map/frontier
      └─ ESDF 点云                        → /ma_nav/map/esdf
```

### 4.3 状态翻转传播链

```
hitPointUpdate / missPointUpdate
  └─ [若 prob 越过阈值导致状态翻转]
      └─ inf_map_->updateGridCounter(from_type, to_type)
          ├─ occupied_cnt-- / unknown_cnt-- (from)
          ├─ occupied_cnt++ / unknown_cnt++ (to)
          └─ [若 grid type 变化]
              └─ InfMap::triggerJumpingEdge(id_g, old_type, new_type)
                  ├─ updateInflation(id_g, ±1)        ← 所有 spherical neighbor
                  │   └─ occ_inflate_cnt[neighbor] ±1
                  └─ updateUnkInflation(id_g, ±1)     ← 同上（若启用）
```

---

## 5. 关键算法

### 5.1 环形缓冲区滑动

地图使用环形缓冲区 3D 索引：
- 局部坐标 `id_l ∈ [-half, half]`
- 哈希 `hash = (id_l+H_x)*Ysize*Zsize + (id_l+H_y)*Zsize + (id_l+H_z)`
- 全局坐标 → 局部坐标：`id_l = id_g % map_size_i`，然后 normalize

滑动时只需清除被覆盖的旧 slice（而非移动内存）。

### 5.2 DDA 射线投射

标准 3D DDA：从传感器原点起步，沿射线方向，每次走到最近的下一个栅格边界，确保不漏格。

### 5.3 多分辨率膨胀计数

不同于对每个细网格做膨胀（O(K×N_neighbor) 内存），CounterMap/InfMap 使用粗分辨率计数器：
- 粗格子 = inflation_ratio³ 个细格子
- 一个细格子翻转为占据 → 粗格子的 occupied_cnt++
- 占据判定：occupied_cnt > 0

### 5.4 Felzenszwalb ESDF

1D 抛物线下包络算法（`fillESDF`），沿 x→y→z 三轴交替传播，正负两遍得到符号距离场。

---

## 6. 性能分析

### 6.1 瓶颈排序（预估）

| 排名 | 瓶颈 | 触发条件 | 每帧开销 |
|------|------|---------|---------|
| 1 | **InfMap::updateInflation** | 大量 cell 状态翻转 | O(K_flipped × N_neighbor) |
| 2 | **ESDFMap::updateESDF3D** | esdf_en=true | O(local_box × 3 维 × 2 遍 × fillESDF) |
| 3 | **ProbMap::raycastProcess** | 点云帧到达 | O(\|cloud\| × ray_steps) |
| 4 | **可视化 boxSearch** | pub_viz_timer_ | O(viz_box_size) ≈ 24M |
| 5 | **MaMap::update_terrain boxSearch** | map_callback | 2 × O(viz_box_size) ≈ 48M |
| 6 | **mapSliding** | 机器人移动超过阈值 | O(shift_slice × 2 维) |
| 7 | **ProbMap::resetLocalMap** | mapSliding 大跳跃 | O(map_vox_num) ≈ 39M |

### 6.2 随历史增长的性能退化

| 退化点 | 原因 |
|--------|------|
| 可视化点云大小 | 占用格数量随探索增加 → `occ_map` 变大 → 序列化/DDS 发送变慢 |
| terrain boxSearch 输出 | 同上 → `real_occ_map` 变大 → terrain analyzer 输入点增多 |
| 地图滑动延迟 | 已探索区域越大，滑动时需清除/翻转的 cell 越多 |
| ESDF 更新 | 翻转 cell 增多 → `triggerJumpingEdge` 触发频率增高 |

### 6.3 典型内存占用

以 `map_size=[100,60,6]`, `resolution=0.1` 为例：
```
prob_map_vox_num = 1001 × 601 × 61 ≈ 36.7M
occupancy_buffer_ (double × 36.7M) ≈ 294 MB
hit_cnt + operation_cnt (int × 2 × 36.7M) ≈ 294 MB
inf_map_vox_num ≈ 1005 × 605 × 65 ≈ 39.5M
inf_map counters (4 × int × 39.5M) ≈ 632 MB
──────────────────────────────────────────
总计 ≈ 1.2 GB+
```

**注意**：当前 `batch_update_size=1`，若增大可减少 ESDF 更新频率但增加延迟。

---

## 7. 已知问题与改进点

### 7.1 高危

| # | 问题 | 位置 | 建议 |
|---|------|------|------|
| 1 | `local_map_origin_i_` 未初始化 | `sliding_map.cpp:initSlidingMap` | ✅ 已修复：始终从 fix_map_origin 初始化 |
| 2 | `MaMap::cfg_` 与 `ROGMap::cfg_` 是两份拷贝 | `ma_map.hpp` | ✅ 已修复：init 前调 setConfig |
| 3 | 新轨迹下发后 `t_track_` 未重置 | `ros2_2d.cpp:plan_omni` | ✅ 已修复：调 reset_track() |

### 7.2 中危（性能）

| # | 问题 | 位置 | 建议 |
|---|------|------|------|
| 4 | viz boxSearch 遍历 24M 格 + 点云随历史变大 | `visualizer.hpp:map_viz_callback` | 加 step 步长参数 + 降采样发布 |
| 5 | `update_terrain` boxSearch 用全 range，GridMap 仅 2m | `ma_map.hpp:update_terrain` | 缩小 box 到 local_update_box [10,10,4] 或更小 |
| 6 | pub_viz_timer_ 100Hz 无意义（实际 ~2Hz） | `ros2_2d.cpp` | ✅ 已修复：加 viz_time_rate 限频 |
| 7 | 无内存使用限制，大地图可能 OOM | `config.hpp:resetMapSize` | 添加 map_vox_num 上限检查 |
| 8 | `boxSearch` 无早期终止 | `prob_map.cpp`, `inf_map.cpp` | 对 KNOWN_FREE 搜索考虑 BFS |

### 7.3 低危（架构/维护）

| # | 问题 | 位置 | 建议 |
|---|------|------|------|
| 9 | `FreeCntMap` 直接继承 `SlidingMap`（有 TODO） | `free_cnt_map.h` | 改为继承 `CounterMap` |
| 10 | `updete_lock` 拼写错误 | `ma_map.hpp` | → `update_lock` |
| 11 | `viz_time_rate` 参数加载但从未使用 | `config.hpp`, `visualizer.hpp` | ✅ 已修复：pub_callback 中限频 |
| 12 | `robot_state_.q` 从未赋值 | `rog_map.cpp:updateRobotState` | ✅ 已修复 |
| 13 | `ROGMap::updateMap` 与 `ProbMap::updateProbMap` 功能重叠 | `rog_map.cpp`, `prob_map.cpp` | 统一入口 |
| 14 | `std::abs(shift_num)` 对 `INT_MIN` 是 UB | `sliding_map.cpp:mapSliding` | 用 `> map_size / < -map_size` 替代 |
| 15 | 膨胀计数溢出风险（int16_t） | `inf_map.h` | 场景极端时检查边界 |

### 7.4 长期演进方向

1. **哈希存储替代密集数组**：省内存（只存非空闲 cell），但 boxSearch 和 serialization 需重新设计。
2. **占用格增量发布**：维护已发布的点云缓存，只在 cell 状态变化时更新，发布频率与历史解耦。
3. **多线程并行**：raycasting / ESDF / viz 可分离到不同线程，利用多核。
4. **自适应分辨率**：机器人附近高分辨率、远处低分辨率，减少全图遍历开销。

---

*文档生成日期：2026-08-07*
*基于 gcopter + ROG-Map（Hong Kong University, MaRS Lab）*
