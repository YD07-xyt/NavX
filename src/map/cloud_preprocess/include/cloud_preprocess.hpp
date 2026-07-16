#pragma once

/**
 * @file cloud_preprocess.hpp
 * @brief 3D 点云预处理核心库（与 ROS2 解耦，可在任意环境下复用）
 *
 * 设计目标（见 README.md）：
 *   1. 离群点滤波（基于 PCL 统计离群点去除）
 *   2. 地面/坡度去除并保留障碍物（基于 2D 栅格高程分析与局部坡度过滤）
 *   3. 过滤悬浮（架空）障碍物（仅保留地面可达高度的障碍）
 *   4. 障碍物点云地图膨胀（可选，输出膨胀后的避障点云地图）
 *   5. 使用 TBB 并行加速栅格处理
 *
 * 对外仅暴露 CloudPreprocess 类与 Params 配置结构，
 * ROS2 节点仅负责消息收发，算法逻辑全部落在本库。
 *
 * 注：使用 PCL 内置 PointXYZ（PCL 仅对内置点类型提供模板实例化），
 *     仅使用 XYZ，不依赖 intensity 字段。
 */

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <Eigen/Core>

#include <string>
#include <vector>

namespace cloud_preprocess {

using PointT = pcl::PointXYZ;        // 仅使用 XYZ，不依赖 intensity 字段
using CloudT = pcl::PointCloud<PointT>;
using CloudPtr = CloudT::Ptr;
using CloudConstPtr = CloudT::ConstPtr;

/// @brief 算法参数集合（与 PARAM_GUIDE.md 一一对应）
struct Params {
  // ---- 离群点滤波 ----
  bool enable_outlier = true;  // 是否开启离群点去除
  int outlier_mean_k = 20;     // 统计邻域点数
  double outlier_std_mul = 1.0;  // 标准差倍数阈值（越大越宽松）

  // ---- 栅格地面分割 ----
  double grid_resolution = 0.1;   // 2D 栅格边长 (m)
  double ground_height = 0.15;   // 高于地面估计值的障碍物阈值 (m)
  double max_height = 3.0;       // 高于此值的点视为噪点/天花板被滤除 (m)
  double min_height = -0.5;      // 低于此值的点视为噪点被滤除 (m)

  // ---- 悬浮障碍物过滤（参考 terrain_analysis：梯度 + 间隙分析）----
  double slope_tolerance = 0.3;    // 邻域地面高度梯度阈值，超过判为垂直障碍(墙/悬崖)
  double suspend_gap = 0.3;        // 柱体内最大垂直间隙超过该值则判为悬浮障碍滤除 (m)
  int ground_median_kernel = 3;    // 地面高度中值平滑核大小(奇数, 1=不平滑)

  // ---- 点云地图膨胀 ----
  bool enable_inflate = false;   // 是否输出膨胀后的障碍物点云地图
  double inflate_radius = 0.3;   // 膨胀半径 (m)，一般取机器人半径

  // ---- 并行 ----
  bool enable_tbb = true;  // 是否使用 TBB 并行处理栅格
};

/// @brief 点云预处理主类
class CloudPreprocess {
 public:
  explicit CloudPreprocess(const Params& params = Params{}) : params_(params) {}

  void setParams(const Params& p) { params_ = p; }
  const Params& params() const { return params_; }

  /**
   * @brief 主处理流程
   * @param input     原始点云（XYZ，不依赖 intensity）
   * @param obstacles 输出：去除地面/坡度后保留的障碍物点云
   * @param ground    输出：被滤除的地面点云
   *
   * 流程：离群点滤波 -> 栅格地面分割（含悬浮过滤）
   */
  void process(const CloudPtr& input, CloudPtr& obstacles, CloudPtr& ground);

  /// @brief 统计离群点滤波，去除悬空噪点
  void removeOutliers(const CloudPtr& input, CloudPtr& output) const;

  /**
   * @brief 基于 2D 栅格的地面分割与障碍提取（核心，参考 terrain_analysis）
   *
   * 思路：
   *   1) 每栅格记录最低点(地面估计)与最高点(顶部)，并对地面做中值平滑；
   *   2) 梯度：计算邻域地面高度差；超过 slope_tolerance 视为垂直障碍(墙/悬崖)，
   *      直接保留（即使悬崖底部未被扫描也能检出，见 3rd.cpp）；
   *   3) 非陡峭栅格：相对地面高于 ground_height 的点，做柱内垂直间隙分析，
   *      若最大间隙 < suspend_gap 说明结构连续(台阶/杆/墙) -> 障碍；
   *      若间隙 >= suspend_gap 说明下方悬空 -> 悬浮障碍，滤除。
   */
  void segmentGround(const CloudPtr& input, CloudPtr& obstacles,
                     CloudPtr& ground) const;

  /**
   * @brief 障碍物点云地图膨胀（可选）
   *
   * 将障碍点栅格化后，以 inflate_radius 在 XY 平面做圆形膨胀，
   * 生成可让地面机器人直接用于避障的膨胀点云地图。
   */
  void inflateCloud(const CloudPtr& obstacles, CloudPtr& inflated) const;

 private:
  Params params_;
};

}  // namespace cloud_preprocess
