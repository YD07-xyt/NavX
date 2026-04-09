// #pragma once
// #include "../base.h"
// #include "../map.h"
// #include "gcopter/minco.hpp"
// #include <cstddef>
// #include <iostream>
// #include <optional>
// namespace planner {

// class TarjOpt {
// private:
//   minco::MINCO_S3NU minco_opt3;
//   std::vector<Point> path_;
//   Trajectory<3> opt_trajectory;
//   bool is_opt = false;
//   Map map_;

// public:
//   TarjOpt(const std::vector<Point> &path, Map map) : path_(path), map_(map) {}

// private:
//   // 降采样路径点
//   std::vector<Point> downsamplePath(const std::vector<Point> &path,
//                                     int step = 5);
//   // 去除重复的相邻点
//   std::vector<Point> removeDuplicates(const std::vector<Point> &path);

//   // 预处理路径：去重 + 降采样
//   std::vector<Point> preprocessPath(const std::vector<Point> &path,
//                                     int downsample_step = 5);

//   Eigen::Matrix3Xd Path2inPoints(const std::vector<Point> &path);

//   // 时间分配
//   Eigen::VectorXd TimeAllocation(const std::vector<Point> &path);

// public:
//   bool optimization(int downsample_step = 5);

//   std::optional<std::vector<Point>> get_traj();

//   // 获取轨迹总时长
//   double getDuration();
// };
// } // namespace planner