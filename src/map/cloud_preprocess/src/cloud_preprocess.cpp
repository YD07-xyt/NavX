#include "cloud_preprocess.hpp"

#include <pcl/filters/statistical_outlier_removal.h>

#include <spdlog/spdlog.h>

#include <tbb/parallel_for.h>

#include <algorithm>
#include <cmath>
#include <unordered_map>
#include <vector>

namespace cloud_preprocess {

namespace {

// 将世界坐标映射到栅格索引键（int32 组合，避免浮点哈希误差）
inline int64_t cellKey(int ix, int iy) {
  return (static_cast<int64_t>(ix) << 32) | (static_cast<uint32_t>(iy));
}
inline void splitKey(int64_t key, int& ix, int& iy) {
  ix = static_cast<int>(key >> 32);
  iy = static_cast<int>(static_cast<uint32_t>(key & 0xFFFFFFFF));
}

// 单栅格统计信息
struct Cell {
  std::vector<float> zs;  // 该栅格内全部点高度
  double min = 0.0;       // 列最低点（未平滑地面估计）
  double max = 0.0;       // 列最高点
  double ground = 0.0;    // 中值平滑后的地面高度
  double grad = 0.0;      // 邻域地面高度梯度
  bool steep = false;     // 是否垂直障碍（墙/悬崖）
  bool blocked = false;   // 柱内结构是否连续（间隙分析）
};

}  // namespace

void CloudPreprocess::process(const CloudPtr& input, CloudPtr& obstacles,
                              CloudPtr& ground) {
  if (!input || input->empty()) {
    spdlog::warn("[CloudPreprocess] 输入点云为空，跳过处理");
    return;
  }

  CloudPtr filtered(new CloudT);
  if (params_.enable_outlier) {
    removeOutliers(input, filtered);
  } else {
    *filtered = *input;
  }

  segmentGround(filtered, obstacles, ground);
}

void CloudPreprocess::removeOutliers(const CloudPtr& input,
                                     CloudPtr& output) const {
  if (!input) return;
  pcl::StatisticalOutlierRemoval<PointT> sor;
  sor.setInputCloud(input);
  sor.setMeanK(params_.outlier_mean_k);
  sor.setStddevMulThresh(params_.outlier_std_mul);
  sor.filter(*output);
  spdlog::debug("[CloudPreprocess] 离群点滤波: {} -> {} 点", input->size(),
                output->size());
}

void CloudPreprocess::segmentGround(const CloudPtr& input, CloudPtr& obstacles,
                                    CloudPtr& ground) const {
  if (!input) return;
  obstacles.reset(new CloudT);
  ground.reset(new CloudT);
  obstacles->header = input->header;
  ground->header = input->header;

  const double res = params_.grid_resolution > 1e-3 ? params_.grid_resolution
                                                    : 0.1;

  // 1) 第一遍：分配点到栅格，记录每格全部高度、最低/最高点
  std::unordered_map<int64_t, Cell> cells;
  for (const auto& p : input->points) {
    if (p.z < params_.min_height || p.z > params_.max_height) continue;
    const int ix = static_cast<int>(std::floor(p.x / res));
    const int iy = static_cast<int>(std::floor(p.y / res));
    const int64_t key = cellKey(ix, iy);
    auto it = cells.find(key);
    if (it == cells.end()) {
      Cell c;
      c.zs.push_back(p.z);
      c.min = c.max = p.z;
      cells.emplace(key, c);
    } else {
      it->second.zs.push_back(p.z);
      if (p.z < it->second.min) it->second.min = p.z;
      if (p.z > it->second.max) it->second.max = p.z;
    }
  }
  if (cells.empty()) return;

  // 2) 地面高度中值平滑（抑制障碍点对地面估计的污染）
  const int k = std::max(1, params_.ground_median_kernel | 1);  // 保证奇数
  const int kr = k / 2;
  for (auto& kv : cells) {
    const int ix = static_cast<int>(kv.first >> 32);
    const int iy = static_cast<int>(static_cast<uint32_t>(kv.first & 0xFFFFFFFF));
    std::vector<double> win;
    win.reserve(k * k);
    for (int dx = -kr; dx <= kr; ++dx) {
      for (int dy = -kr; dy <= kr; ++dy) {
        auto nit = cells.find(cellKey(ix + dx, iy + dy));
        if (nit != cells.end()) win.push_back(nit->second.min);
      }
    }
    std::sort(win.begin(), win.end());
    kv.second.ground = win[win.size() / 2];
  }

  // 3) 邻域梯度（8 邻域地面高度差最大值）
  for (auto& kv : cells) {
    const int ix = static_cast<int>(kv.first >> 32);
    const int iy = static_cast<int>(static_cast<uint32_t>(kv.first & 0xFFFFFFFF));
    double max_diff = 0.0;
    for (int dx = -1; dx <= 1; ++dx) {
      for (int dy = -1; dy <= 1; ++dy) {
        if (dx == 0 && dy == 0) continue;
        auto nit = cells.find(cellKey(ix + dx, iy + dy));
        if (nit == cells.end()) continue;
        max_diff = std::max(max_diff,
                            std::abs(kv.second.ground - nit->second.ground));
      }
    }
    kv.second.grad = max_diff;
    kv.second.steep = max_diff > params_.slope_tolerance;
  }

  // 4) 柱内垂直间隙分析：排序高度，求相邻最大间隙
  for (auto& kv : cells) {
    Cell& c = kv.second;
    std::vector<float> hs;
    hs.reserve(c.zs.size());
    for (float z : c.zs) {
      if (z >= c.ground - 0.05f) hs.push_back(z);  // 过滤过低噪声
    }
    if (hs.empty()) {
      c.blocked = false;
      continue;
    }
    std::sort(hs.begin(), hs.end());
    float last = c.ground;
    float max_gap = 0.0f;
    for (float z : hs) {
      if (z <= last) continue;
      max_gap = std::max(max_gap, z - last);
      last = z;
    }
    // 连续（最大间隙 < suspend_gap）且顶部高于地面 -> 视为实体障碍
    c.blocked = (max_gap < params_.suspend_gap) &&
                (c.max > params_.ground_height);
  }

  // 5) 逐点分类
  const auto classify = [&](const PointT& p) -> int {
    if (p.z < params_.min_height || p.z > params_.max_height) return -1;  // 噪点
    const int ix = static_cast<int>(std::floor(p.x / res));
    const int iy = static_cast<int>(std::floor(p.y / res));
    auto it = cells.find(cellKey(ix, iy));
    if (it == cells.end()) return 0;  // 无数据栅格 -> 视为地面
    const Cell& c = it->second;

    if (c.steep) return 1;  // 垂直障碍（墙/悬崖），即使底部未扫描也保留
    const double rel = p.z - c.ground;
    if (rel <= params_.ground_height) return 0;        // 地面
    return c.blocked ? 1 : -1;  // 高于地面：连续结构->障碍；悬空->滤除
  };

  if (params_.enable_tbb) {
    std::vector<int> labels(input->size(), 0);
    tbb::parallel_for(size_t(0), input->size(), [&](size_t i) {
      labels[i] = classify(input->points[i]);
    });
    ground->reserve(input->size());
    obstacles->reserve(input->size());
    for (size_t i = 0; i < input->size(); ++i) {
      if (labels[i] < 0) continue;
      (labels[i] == 0 ? *ground : *obstacles).points.push_back(
          input->points[i]);
    }
  } else {
    for (const auto& p : input->points) {
      const int lab = classify(p);
      if (lab < 0) continue;
      (lab == 0 ? *ground : *obstacles).points.push_back(p);
    }
  }
  ground->width = static_cast<uint32_t>(ground->size());
  ground->height = 1;
  obstacles->width = static_cast<uint32_t>(obstacles->size());
  obstacles->height = 1;

  spdlog::debug("[CloudPreprocess] 地面分割: 地面 {} / 障碍 {}", ground->size(),
                obstacles->size());
}

void CloudPreprocess::inflateCloud(const CloudPtr& obstacles,
                                   CloudPtr& inflated) const {
  if (!obstacles) return;
  inflated.reset(new CloudT);
  inflated->header = obstacles->header;

  const double res = params_.grid_resolution > 1e-3 ? params_.grid_resolution
                                                    : 0.1;
  const int r = std::max(1, static_cast<int>(std::ceil(
                               params_.inflate_radius / res)));
  const int r2 = r * r;

  // 1) 障碍点栅格化：每格记录最低障碍高度
  std::unordered_map<int64_t, float> grid;
  for (const auto& p : obstacles->points) {
    const int ix = static_cast<int>(std::floor(p.x / res));
    const int iy = static_cast<int>(std::floor(p.y / res));
    const int64_t key = cellKey(ix, iy);
    auto it = grid.find(key);
    if (it == grid.end() || p.z < it->second) grid[key] = p.z;
  }

  // 2) 以 inflate_radius 在 XY 平面做圆形膨胀
  std::unordered_map<int64_t, float> inflated_grid;
  for (const auto& kv : grid) {
    int ix, iy;
    splitKey(kv.first, ix, iy);
    const float z = kv.second;
    for (int dx = -r; dx <= r; ++dx) {
      for (int dy = -r; dy <= r; ++dy) {
        if (dx * dx + dy * dy > r2) continue;  // 圆形膨胀
        const int64_t nk = cellKey(ix + dx, iy + dy);
        auto it = inflated_grid.find(nk);
        if (it == inflated_grid.end() || z < it->second) inflated_grid[nk] = z;
      }
    }
  }

  // 3) 生成膨胀点云（栅格中心点）
  inflated->reserve(inflated_grid.size());
  for (const auto& kv : inflated_grid) {
    int ix, iy;
    splitKey(kv.first, ix, iy);
    PointT pt;
    pt.x = static_cast<float>((ix + 0.5) * res);
    pt.y = static_cast<float>((iy + 0.5) * res);
    pt.z = kv.second;
    inflated->points.push_back(pt);
  }
  inflated->width = static_cast<uint32_t>(inflated->size());
  inflated->height = 1;
  spdlog::debug("[CloudPreprocess] 膨胀地图: {} -> {} 栅格点", grid.size(),
                inflated->size());
}

}  // namespace cloud_preprocess
