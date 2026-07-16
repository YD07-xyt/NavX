# 参数变更记录 (PARAM_GUIDE)

> 每次修改算法参数或新增参数都在此追加记录，并同步更新 `config/cloud_preprocess.yaml`
> 与 `include/cloud_preprocess.hpp` 中 `Params` 结构体默认值。

## 参数总览

| 参数 | 类型 | 默认 | 说明 |
| --- | --- | --- | --- |
| `input_cloud_topic` | string | ~/input_cloud | 输入点云话题(PointCloud2) |
| `input_cloud_custom_topic` | string | ~/input_cloud_custom | 输入点云话题(livox_ros_driver2 CustomMsg，含每点 time_offset) |
| `input_imu_topic` | string | ~/input_imu | 输入 IMU 话题 |
| `input_odom_topic` | string | ~/input_odom | 输入里程计话题 |
| `obstacles_topic` | string | ~/obstacles | 障碍物点云输出话题 |
| `ground_topic` | string | ~/ground | 地面点云输出话题 |
| `inflated_topic` | string | ~/inflated | 膨胀点云地图输出话题（enable_inflate 时生效） |
| `enable_distortion` | bool | false | 是否开启运动去畸变（需 PointCloud2 含每点 time 字段） |
| `time_scale` | double | 1.0 | PointCloud2 每点 time 字段换算为“秒”的系数(如 ns 填 1e-9) |
| `custom_time_scale` | double | 1e-9 | CustomMsg time_offset(ns) 换算为“秒”的系数 |
| `enable_outlier` | bool | true | 是否开启统计离群点去除 |
| `outlier_mean_k` | int | 20 | 离群点统计邻域点数 |
| `outlier_std_mul` | double | 1.0 | 离群点标准差倍数阈值（越大越宽松） |
| `grid_resolution` | double | 0.1 | 2D 栅格边长 (m)，越小越精细但越慢 |
| `ground_height` | double | 0.15 | 相对地面高于该值视为障碍物 (m) |
| `max_height` | double | 3.0 | 高于此值的点视为噪点/天花板被滤除 (m) |
| `min_height` | double | -0.5 | 低于此值的点视为噪点被滤除 (m) |
| `slope_tolerance` | double | 0.3 | 邻域地面高度梯度阈值，超过判为垂直障碍(墙/悬崖) |
| `suspend_gap` | double | 0.3 | 柱体内最大垂直间隙超过该值，判为悬浮障碍滤除 (m) |
| `ground_median_kernel` | int | 3 | 地面高度中值平滑核大小(奇数, 1=不平滑) |
| `enable_inflate` | bool | false | 是否输出膨胀后的障碍物点云地图 |
| `inflate_radius` | double | 0.3 | 膨胀半径 (m)，一般取机器人半径 |
| `enable_tbb` | bool | true | 是否使用 TBB 并行处理栅格 |
| `enable_accumulate` | bool | false | 是否累计多帧“输入”点云后再统一处理(稠密化) |
| `accumulate_range` | double | 5.0 | 累计点云在机器人周围保留半径 (m) |
| `accumulate_lifetime` | double | 2.0 | 累计点云点存活时间 (s)，超出则衰减移除 |

## 变更记录

### v0.0.8 (支持 livox CustomMsg 输入)
- 新增 `livox_ros_driver2/msg/CustomMsg` 订阅(`input_cloud_custom_topic`)，其每点 `time_offset`
  直接用于运动去畸变(换算系数 `custom_time_scale`，默认 1e-9 即 ns->s)；PointCloud2 与 CustomMsg
  两种输入均可使用，去畸变是否生效取决于是否有每点时间戳。
- `livox_ros_driver2` 作为可选依赖：安装后自动开启 CustomMsg 支持(HAVE_LIVOX)，
  未安装则仅 PointCloud2 输入(编译期自动关闭 CustomMsg 路径)。

### v0.0.7 (运动去畸变修正)
- 修正 `deDistort` 数学：每点按相对时间 t 做 R(-w*t)*(p-v*t) 反投影，t=0 为单位变换；
  无每点时间戳时安全退化为透传（不再错误旋转整帧）。
- 新增从 PointCloud2 的 time/t/time_offset/timestamp 字段提取每点时间戳（节点 `extractTimes`），
  并新增 `time_scale` 参数做单位换算；livox/mid360 需保证每点时间随点云传入。

### v0.0.6 (累计输入点云：稠密化静态障碍)
- 新增 `enable_accumulate` / `accumulate_range` / `accumulate_lifetime`：开启后在机器人周围
  accumulate_range 范围内累计多帧“输入”点云，并随时间衰减(超过 lifetime 移除)，
  再对累计点云统一跑完整流程(去噪+地面分割)，使地面估计更稳、墙壁更稠密；
  关闭则逐帧实时处理单帧输入。

### v0.0.5 (悬浮过滤改为 terrain_analysis 式：梯度+间隙分析)
- 参照 3rd.cpp：邻域地面高度梯度超过 `slope_tolerance` 直接判为垂直障碍(墙/悬崖)，
  即使悬崖底部未扫描也可保留；非陡峭高点为柱内垂直间隙分析，`suspend_gap` 内连续则保留，
  存在大间隙(悬空)则滤除。新增 `suspend_gap` / `ground_median_kernel` 参数。

### v0.0.4 (悬浮过滤改为结构连通性判定)
- 悬浮障碍改为基于连通性判定：本列有向下支撑或邻近栅格存在同高度结构则保留；
  悬空于周边地形之上且无支撑的点（架空/悬浮障碍）滤除。
- 悬崖/墙体/杆等连通结构不再被误删；移除了原 `obstacle_max_height` 参数。

### v0.0.3 (膨胀地图 + 悬浮过滤)
- 新增 `enable_inflate` / `inflate_radius` 与 `inflated_topic`：可选输出
  XY 平面圆形膨胀后的障碍物点云地图，供地面机器人直接避障。

### v0.0.2 (话题可配置 + 去 intensity 依赖)
- 改用 `pcl::PointXYZ`，输入点云不再依赖 intensity 字段。
- 新增 `input_*_topic` / `*_topic` 参数，话题名可在 yaml 中配置。

### v0.0.1 (初始化)
- 新建 `Params` 结构体，定义全部基础参数及默认值；
- 完成去畸变（可选）、离群点滤波、栅格地面分割三大功能；
- 新增 `config/cloud_preprocess.yaml` 与节点参数声明。
