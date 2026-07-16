# Terrain Map 稳定高频发布 — 改动说明

## 问题
`terrain_map`（话题 `rog_map/terrain_map`）在 RViz 中观察有轻微抖动，发布不够稳定。

## 根因
`TerrainAnalyzer::setupGrid` 以机器人当前位姿直接计算分析窗口原点
（`robot_pos - map_size/2`）。由于机器人连续运动，窗口原点发生**亚网格级（sub-resolution）**
连续偏移，导致每个网格中心的世界坐标每帧都轻微变化，发布出来的地形点随之抖动。

## 改动

### 1. 消除地形点抖动（核心修复）
文件：`src/terrain_analysis/terrain_analysis.cpp` — `setupGrid`

将分析窗口原点对齐（snap）到 `resolution` 网格上：

```cpp
const float res = config_.resolution;
const float ox = std::round((robot_pos.x() - config_.map_size_x * 0.5f) / res) * res;
const float oy = std::round((robot_pos.y() - config_.map_size_y * 0.5f) / res) * res;
grid.origin = rog_map::Vec3f(ox, oy, 0.0f);
```

网格中心在世界坐标系下变为固定位置，只有当机器人跨越一个 `resolution` 单元时窗口
才会整体平移一格，避免了连续抖动。窗口相对机器人的最大偏移不超过 `resolution/2`，
不影响地形分析的正确性。

### 2. 稳定且可配置的高频发布
- `include/rog_map/rog_map_core/config.hpp`：新增配置项
  `double terrain_time_rate{50.0};`（单位 Hz），并在 `LoadYaml` 中加载
  `terrain/time_rate`（默认值 50.0）。
- `include/rog_map_ros/rog_map_ros2.hpp`：地形定时器周期由原来硬编码的
  `20ms` 改为由 `cfg_.terrain_time_rate` 计算
  （`1000 / max(terrain_time_rate, 1.0)` ms），默认即 50Hz。
- `config/rog_map.yaml`：新增
  ```yaml
  terrain:
    time_rate: 50.0   # terrain_map 发布频率(Hz)
  ```

## 使用
- 默认发布频率为 **50 Hz**。如需更高频率，调大 `config/rog_map.yaml` 中的
  `terrain.time_rate`（如 `100.0` 对应 10 ms）。
- 注意：频率越高，单线程执行器下地形分析（`analyze`）的 CPU 占用越大；
  请确保 `1 / terrain_time_rate` 大于单次 `analyze` 的实际耗时，否则实际发布
  周期会被分析耗时拖长。

## 影响范围
仅改动 terrain_map 相关链路（地形分析网格对齐 + 发布定时器），未改动 occ / esdf /
frontier / 可视化边界等其他模块。

---

# 全局地图发布（PCD → 3docc + terrain_map）— 改动说明

## 问题
需要从 PCD 文件读取完整全局地图，并以两个独立话题发布：
- `rog_map/global_3docc`：完整原始 PCD 点云（3D 占据点）
- `rog_map/global_terrain_map`：基于完整 PCD 的地形分析结果

该功能需在不影响原有 `rog_map/occ`、`rog_map/terrain_map`、`load_pcd_en`
等本地地图/可视化链路的前提下叠加。

## 改动

### 1. 配置扩展
- `config/rog_map.yaml`：新增
  ```yaml
  global_map:
    enable: false
    pcd_path: "/path/to/global_map.pcd"
    terrain_origin: [0.0, 0.0, 0.0]
  ```
- `include/rog_map/rog_map_core/config.hpp`：
  - 新增 `bool global_map_en{false};`
  - 新增 `string global_map_pcd_path{};`
  - 新增 `Vec3f global_map_terrain_origin{};`
  - 在 `Config::Config(...)` 中加载上述参数，`terrain_origin` 按 `vector<double>`
    解析并校验长度为 3。

### 2. ROS2 接口新增
- `package.xml`：新增依赖 `<depend>std_srvs</depend>`
- `CMakeLists.txt`：新增 `find_package(std_srvs REQUIRED)` 并加入 `ros_libs`
- `include/rog_map_ros/rog_map_ros2.hpp`：
  - 新增 `std_srvs/srv/Trigger` 服务 `/rog_map/publish_global_map`
  - 新增两个发布器：
    - `rog_map/global_3docc`（`sensor_msgs/PointCloud2`）
    - `rog_map/global_terrain_map`（`sensor_msgs/PointCloud2`）

### 3. 核心逻辑：初始化读 PCD + 持续发布
- `include/rog_map_ros/rog_map_ros2.hpp`：
  - 新增 `loadGlobalPCD()`：在构造函数中调用一次，读取 PCD 并完成地形分析，
    结果缓存到 `vm_.global_pcd_map_` 和 `vm_.global_terrain_map_`。
  - 新增 `publishGlobalMap()`：仅发布缓存数据，不重复加载/计算。
  - 新增 `global_map_timer_`：按 `cfg_.global_map_time_rate` 周期调用
    `publishGlobalMap()`，实现持续发布。
  - 移除 `std_srvs/srv/Trigger` 服务，不再需要手动调用。

## 使用
1. 在 `config/rog_map.yaml` 中启用：
   ```yaml
   global_map:
     enable: true
     pcd_path: "/path/to/your/global_map.pcd"
     time_rate: 1.0   # 发布频率(Hz)，默认 1Hz
   ```
2. 启动节点后，全局地图将在初始化时自动读入 PCD 并持续发布：
   - `rog_map/global_3docc`
   - `rog_map/global_terrain_map`

## 影响范围
- 默认 `global_map.enable: false`，不创建新话题/服务，不加载 PCD，零额外开销。
- 仅在启用后，初始化时读取一次指定 PCD，缓存后按 `time_rate` 持续发布两个新话题。
- 原有 `load_pcd_en`、`rog_map/occ`、`rog_map/terrain_map`、ESDF、frontier
  等本地地图/可视化链路完全独立，不受影响。

---

# TerrainAnalyzer 参数 YAML 化 — 改动说明

## 问题
`TerrainAnalyzer` 的配置参数（`kernel_size`、`max_step_height`、`robot_height`、
`steep_threshold`）在代码中硬编码，无法通过配置文件调整。

## 改动
- `config/rog_map.yaml`：在 `terrain:` 下新增可配置项
  ```yaml
  terrain:
    time_rate: 30.0
    kernel_size: 5                     # 中值滤波核大小
    max_step_height: 0.15              # 台阶/墙体最小高度差（用于触发间隙分析）
    robot_height: 0.3                  # 机器人安全通行所需的最小垂直间隙
    steep_threshold: 0.17              # 陡峭边缘阈值，用于触发垂直障碍判定
  ```
- `include/rog_map/rog_map_core/config.hpp`：
  - 新增成员变量：`terrain_kernel_size`、`terrain_max_step_height`、
    `terrain_robot_height`、`terrain_steep_threshold`
  - 在 `Config::Config(...)` 中加载上述参数
- `include/rog_map_ros/rog_map_ros2.hpp`：
  - 本地 `terrain_analyzer_` 初始化时使用 yaml 参数
  - 全局地图 `loadGlobalPCD()` 中的临时 `TerrainAnalyzer` 也使用同一套 yaml 参数

## 使用
修改 `config/rog_map.yaml` 中的 `terrain.*` 参数即可调整地形分析行为，
无需重新编译。

---

# 动态障碍物清除 — 占据时间衰减 — 改动说明

## 问题
原概率占据栅格（`occupancy_buffer_`）的更新只有两条单向路径：
- `hitPointUpdate`（prob_map.cpp）：`ret += l_hit * hit_num`，只增不减，封顶 `l_max`
- `missPointUpdate`（prob_map.cpp）：`ret += l_miss * hit_num`，只减不增

每个 cell **没有时间戳、没有衰减**。动态障碍（行人 / 车辆 / 其他机器人）经过时把格子打到
`l_max` 占据；它离开后：
1. 若身后没有更远的表面，射线再也不会穿过该格 → 永远卡在占据（"鬼影/残影"）；
2. 即使身后的墙能清除，因为 `l_hit >> |l_miss|`，也需要持续大量 free-ray 才能压回，清除很慢。

结果：残影长期阻挡规划路径，而静态障碍本应保留 —— 原框架**无法区分静/动态**。

## 改动思路
引入**占据时间衰减**：每个 cell 记录最后一次被观测（命中或穿过 free-ray）的墙钟时间；
若某占据（或未知）格超过 `occ_decay_time` 秒未被再次观测，就按 `occ_decay_rate` 倍 `l_miss`
向其自由方向衰减。离开的动态障碍在数秒内自动淡出，而持续被观测的静态障碍稳态仍占高，不受影响。

为与 ROG-Map 的环形（toroidal）滑动窗口缓冲保持一致，每个 buffer slot 记录自己的
`last_obs_time_`：地图滑动时只有"移出窗口的条带"经 `resetCell` 清零，窗口内其余 slot
的 `last_obs_time_` 随占据值一起随 slot 平移，语义自洽。

## 代码改动
- `include/rog_map/rog_map_core/config.hpp`
  - 新增参数 `occ_decay_time`（默认 `0.0`，秒；`<=0` 表示关闭衰减）
  - 新增参数 `occ_decay_rate`（默认 `1.0`，每次衰减施加的 `l_miss` 倍数）
  - 在 `Config` 构造函数中加载 `rog_map/raycasting/occ_decay_time` 与
    `rog_map/raycasting/occ_decay_rate`
- `include/rog_map/prob_map.h`
  - 新增缓冲 `std::vector<float> last_obs_time_;`（0 表示该格从未被观测）
  - 新增 `double cur_wall_time_;`，在 `updateProbMap` 开头写入当前墙钟时间
  - 新增成员函数 `void decayOccupancy();`
  - 新增虚函数 `virtual const double getSystemWalltimeNow()`（默认实现用
    `std::chrono` 单调时钟；ROS 层 `RogMapRos` 已覆盖为 ROS 时钟）
- `src/rog_map/prob_map.cpp`
  - `initProbMap`：`last_obs_time_` 随 `occupancy_buffer_` 一起分配
  - `updateProbMap`：开头取 `cur_wall_time_ = getSystemWalltimeNow()`；在
    `probabilisticMapFromCache()` 之后调用 `decayOccupancy()`
  - `hitPointUpdate` / `missPointUpdate`：更新该 cell 时写入 `last_obs_time_[hash_id]`
  - `resetCell` / `resetLocalMap`：相应清零 `last_obs_time_`
  - 新增 `decayOccupancy()`：遍历整张 local map，对"曾被观测、当前非自由、且
    `cur_wall_time_ - last_obs_time_ > occ_decay_time`"的格子施加一次 miss 衰减；类型翻转时
    回调 `inf_map_->updateGridCounter` / `esdf_map_->updateGridCounter` /
    `fcnt_map_->updateFrontierCounter`，保证膨胀层与 ESDF 同步
- `config/rog_map.yaml`：在 `raycasting:` 下新增
  ```yaml
  occ_decay_time: 5.0     # 衰减触发的时间阈值(s), 0=关闭
  occ_decay_rate: 1.0     # 衰减强度 (每次施加的 l_miss 倍数)
  ```

## 使用与调参
- 默认 `occ_decay_time: 0.0`，即**完全关闭**，行为与改动前一致。
- 开启动态障碍清除：把 `occ_decay_time` 设为一个略大于"传感器对一处的典型重观测间隔"的值
  （如 `3.0 ~ 10.0` 秒）。动态障碍离开后约 `occ_decay_time` 秒开始淡出，再经若干帧降到自由。
- 若静态障碍（墙、柱）也被误清，说明 `occ_decay_time` 太小或 `occ_decay_rate` 太大，
  调大 `occ_decay_time` 或调小 `occ_decay_rate`。
- 性能：`decayOccupancy` 每 `batch_update_size` 帧全量扫描一次 local map（与 `raycasting` 同频次），
  复杂度为 O(地图体素数)，对常规地图开销可忽略；如需更省，可增大 `batch_update_size`。

## 影响范围
- 仅改动概率占据层（`ProbMap`）的更新与衰减逻辑，命中/未命中概率更新本身不变。
- `occ_decay_time <= 0` 时 `decayOccupancy` 直接返回，零额外开销，完全兼容原行为。
- 膨胀层、ESDF、frontier、地形分析、可视化等模块均未改动接口，仅随占据翻转被动更新计数器。

