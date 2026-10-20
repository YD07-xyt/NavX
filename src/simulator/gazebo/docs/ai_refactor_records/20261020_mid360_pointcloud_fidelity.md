# Mid-360 仿真点云保真度改造记录（距离偏差 / 量程 / 噪声 / 每点时间 / always_on）

## User Intent

用户先问“输出点云的质量由什么决定，怎么提升仿真点云质量”，在给出分析后**明确选择方案 B：改插件 C++**。
目标：让仿真点云的**距离模型与时间模型**和实物 Livox Mid-360 一致，用于 Super-LIO 等里程计仿真/回归。

## Scope

允许修改：

- `livox_laser_simulation_RO2/src/livox_points_plugin.cpp`
- `livox_laser_simulation_RO2/include/ros2_livox/livox_points_plugin.h`
- `livox_laser_simulation_RO2/urdf/mid360.xacro`（注释 + `<always_on>` 修复）

## Out of Scope

本次明确不做：

1. 不改 `samples` / `downsample` / `update_rate`（20000 点/帧 × 10 Hz = 20 万点/秒，与实物标称一致）。
2. 不改 world、不改地图 STL、不补 `laser_retro`。
3. 不补 intensity / reflectivity 模型（真实反射率依赖材质，仿真侧无依据，硬编等于伪造数据）。
4. 不改 PointCloud2 的 `rgb` 字段（仍只填 xyz；下游应使用 CustomMsg）。
5. 不把无效点从 `(0,0,0)` 改成“丢弃”（实物驱动就是按 0 发布，保持现状更真实）。
6. 不改 `CMakeLists.txt`。
7. 不改 Super-LIO 侧参数（`filter_rate` / `blind` / `maxrange`）。

## Explorer Findings

### Files inspected

- `livox_laser_simulation_RO2/src/livox_points_plugin.cpp`（改前 402 行）、`include/ros2_livox/livox_points_plugin.h`
- `livox_laser_simulation_RO2/src/livox_ode_multiray_shape.cpp`（`Init()` 覆盖了基类 `Init()` 且未调用基类）
- `livox_laser_simulation_RO2/urdf/mid360.xacro`、`scan_mode/mid360.csv`（800000 行，Time/s = 行号 µs）
- `pb_rm_simulation/urdf/simulation_waking_robot.xacro`、`launch/rm_simulation.launch.py`
- `pb_rm_simulation/world/RMUC2027_world/RMUC2027_world.world`（全场景只有 1 个 map 模型 / 1 个 trimesh collision）
- `pb_rm_simulation/meshes/RMUC2027_world/meshes/rmuc_2027.stl`（378408 三角形、1617.6 m²）
- `/usr/include/gazebo-11/gazebo/physics/{MultiRayShape,RayShape,Sensor}.hh`、`/usr/share/sdformat12/*/sensor.sdf`
- `/usr/lib/x86_64-linux-gnu/libgazebo_physics.so`（反汇编，非推测）
- Super-LIO：`src/ros/ROSWrapper.cpp`（`livoxHandler`）、`src/lio/super_lio.cpp`（`Propagation_Undistort`）

### Active logic path

```
mid360.csv(800k 行) --<samples>=20000--> 每帧 20000 条 ODE 射线（currStartIndex += 20000）
  -> rayShape->Update() -> MultiRayShape::Update()
       (每条射线 SetLength(maxRange - minRange)，再 dSpaceCollide2 求交)
  -> LivoxOdeMultiRayShape::UpdateCallback: 命中时 shape->SetLength(contact.depth)
  -> OnNewLaserScans(): GetRange() = MultiRayShape::minRange + rayLength
  -> /livox/lidar (CustomMsg) + /livox/lidar/pointcloud (PointCloud2, rgb 全 0)
```

### Data flow（反汇编确认的 Gazebo 内部语义）

1. `MultiRayShape::GetRange(i)` @0x27aac0：`range = GetMinRange() + RayShape::GetLength()`，无 max clamp。
2. `MultiRayShape::Init()` @0x279de0 是**唯一**写私有成员 `minRange`(+0x250) / `maxRange`(+0x258) 的函数。
3. `LivoxOdeMultiRayShape::Init()` 是基类 `Init` 的拷贝、删掉了建射线循环，把 `this->minRange` 改成**局部变量**
   （基类成员 private，改写会编译失败），且**不调用基类 `Init()`** → 运行期 `minRange=0`、`maxRange=1000`。
4. `MultiRayShape::Update()` @0x27b040：每条射线 `SetLength(GetMaxRange()-GetMinRange())` = 1000；
   `ODERayShape::SetPoints()` 不写 `RayShape::length`，只有命中时 `SetLength(depth)` 才写。
5. 插件建射线时起点在 `minDist * axis`（离传感器 0.1 m）。

确定性结论：

- 命中：`GetRange() = 0 + contact.depth = 真实距离 - minDist` → **所有点系统性近 10 cm**（本次已修）。
- 未命中：`GetRange() = 1000` → 恰好被旧代码 `range >= RangeMax()(=1000)` 抓到（靠两个 1000 相等）。
- `<range><max>40` 对求交无效（射线被重置为 1000 m）；`<range><resolution>`、`<noise>`、`<scan>` 采样数
  对点云全部无效（读的是同样未被写入的成员；`<noise>` 只在 `sensors::RaySensor` 内部生效）。

### Risk notes

- 每点时间改变后一帧 `end_time` 从 `stamp+几 ms` 变为 `stamp+100 ms`，Super-LIO `sync_measure()` 会多等约 1 帧 IMU，
  输出延迟 +1 帧 —— 这是对齐实物扫描周期的代价。
- 噪声固定随机种子（5489），保证同一场景仿真可复现。
- **验证阶段新发现**：`<always_on>` 在 SDF 中默认 **false**（`/usr/share/sdformat12/1.8/sensor.sdf`），雷达 sensor 没写它，
  所以**只有开着 gzclient 时才被激活**；无头跑 `gzserver` 时点云完全不更新（本次实测：200 s 一帧都没有）。

## Modifier Changes

### Files changed

1. `src/livox_points_plugin.cpp`
2. `include/ros2_livox/livox_points_plugin.h`
3. `urdf/mid360.xacro`

### Key changes

1. **距离修正 + 量程截断**：
   ```cpp
   double distance = 0.0;
   if (depth > RangeMin() && depth < RangeMax()) {   // 剔掉未命中哨兵(1000)
       distance = depth + minDist;                   // 补回射线起点的 minDist
       if (range_noise_enabled_) distance += range_noise_(rng_);
       if (distance <= 0.0 || distance >= maxDist) distance = 0.0;  // 按 <range><max>=40 截断
   }
   auto point = distance * axis;
   ```
2. **测距高斯噪声**：`Load()` 读 `<ray><noise>` 的 `type/mean/stddev`，仅 `gaussian` 且 `stddev>0` 时启用，
   每个有效点加一次 `N(mean, stddev)`。原 xacro 里这段是死配置，现在生效。
3. **每点时间偏移**：删掉 `boost::chrono` 墙钟（原来只有 ms 级），改为
   `offset_time = 帧内序号 × (1e9/update_rate/N)`，一帧 20000 点均匀铺满 100 ms 扫描周期。
4. 头文件新增 `std::mt19937 rng_{5489u}`、`std::normal_distribution<double> range_noise_`、`bool range_noise_enabled_`，`#include <random>`。
5. **`mid360.xacro` 新增 `<always_on>true</always_on>`**（无头仿真必须），并把“插件不读 `<noise>`”的过时注释改掉。

### Behavior preserved

- 话题、frame_id、QoS、10 Hz 帧率、每帧 20000 点、射线方向、0.1 m 盲区全部不变。
- 无效点仍按 `(0,0,0)` 发布；`laserscan`（`scanPub`）话题行为不变；`reflectivity` 仍来自 `GetRetro()`（恒 0）。

### Behavior intentionally adjusted

1. 命中点沿视线**外移 0.1 m**，恢复真实距离。
2. 距离 ≥ 40 m（`<range><max>`）的点被截断为无效点。
3. 距离带 σ=2 cm 高斯抖动。
4. 每帧 `offset_time` 从 0~几 ms 变为 0~99.995 ms。
5. 无头（无 gzclient）运行时雷达现在也会更新（此前完全不更新）。

## Auditor Review

### Checks performed

- [x] 关键路径检查：`Load()` 读 `<noise>`、`OnNewLaserScans()` 距离/噪声/时间三段，逐行复核
- [x] diff 检查：3 个文件 +67/−13（不含记录），全部落在预期文件
- [x] grep 检查：`boost::chrono` / `start_time` 无残留；`RangeMin()/RangeMax()/minDist/maxDist/rng_` 与头文件一致
- [x] XML / xacro 检查：两个 xacro `xml.etree` 解析通过；`xacro` 展开后插件 SDF 含 `<noise>` σ=0.02、`<always_on>` 生效
- [x] 构建：`colcon build --packages-select ros2_livox_simulation --symlink-install` → **通过**（11.3 s，exit 0）
- [x] 运行期验证（无头 gzserver + RMUC2027 + spawn robot，见下）

### Runtime verification（实测数据）

| 项 | 结果 |
|---|---|
| 插件加载 | `sample: 20000`、`downsample: 1`、`range noise: type gaussian mean 0.0000 stddev 0.0200 enabled 1` |
| 每帧点数 / 频率 | `point_num=20000`，帧间隔 0.12 s（≈ 10 Hz 仿真时间，RTF≈0.83） |
| `offset_time` | min=0，max=**99,995,000 ns**，单调递增 → 一帧铺满 100 ms ✔ |
| `always_on` 前后 | 修复前无头跑 200 s **0 帧**；修复后 0.2 s 内出帧 ✔ |
| 距离 vs STL 真值 | 用 STL（378k 三角形）+ 传感器位姿做 Möller-Trumbore 正向求交，对 201 条非掠射射线拟合 `残差 = b + δ·d̂`：**b = −0.0026 ± 0.0020 m**（旧代码应为 +0.1000）、位姿修正 (+0.016, +0.001, +0.029) m、残差 std **0.026 m**（≈ 设定 σ=0.02 + 几何误差）→ 10 cm 偏差已消除、噪声已生效 ✔ |
| 其余分布 | 有效点 4145/20000，失效 `(0,0,0)` 15855（射线朝上无天花板）；`d_min=0.75 m`、`d_max=25.4 m`（< 40 m） |

### Issues found

1. ~~未构建~~ → 已构建并通过。
2. **仍是死参数**（未改，仅记录）：`<scan>` 的 100/360 采样与角度、`<range><resolution>` 对点云无效；
   `mid360.xacro` 里的 `samples` 属性（30000）也是死参数（插件用字面值 20000）。
3. `sensor/<visualize>true</visualize>` 仍开着（GUI 会画 2 万条射线，可能拖慢 gzclient），本次未改。
4. intensity 恒 0 / PointCloud2 `rgb` 全 0（范围内已排除）。
5. `CMakeLists.txt` 里的 `libprotobuf.so.9` / `libboost_chrono.so.1.71.0` 是 20.04 库名（本机 22.04），本次构建未受影响，未改动。

### Final result

PASS
