// #include "../../include/trajectory_optimization/trajopt.h"
// namespace planner {

// // 降采样路径点
// std::vector<Point> TarjOpt::downsamplePath(const std::vector<Point> &path,
//                                            int step) {
//   if (path.size() <= step)
//     return path;

//   std::vector<Point> downsampled;
//   downsampled.reserve(path.size() / step + 1);

//   for (size_t i = 0; i < path.size(); i += step) {
//     downsampled.push_back(path[i]);
//   }
//   // 确保包含终点
//   if (downsampled.back().x != path.back().x ||
//       downsampled.back().y != path.back().y) {
//     downsampled.push_back(path.back());
//   }

//   std::cout << "降采样: " << path.size() << " -> " << downsampled.size()
//             << " 点" << std::endl;
//   return downsampled;
// }

// // 去除重复的相邻点
// std::vector<Point> TarjOpt::removeDuplicates(const std::vector<Point> &path) {
//   if (path.empty())
//     return {};

//   std::vector<Point> cleaned;
//   cleaned.reserve(path.size());
//   cleaned.push_back(path[0]);

//   for (size_t i = 1; i < path.size(); i++) {
//     double dx = std::abs(path[i].x - cleaned.back().x);
//     double dy = std::abs(path[i].y - cleaned.back().y);
//     if (dx > 1e-6 || dy > 1e-6) {
//       cleaned.push_back(path[i]);
//     }
//   }

//   if (cleaned.size() < path.size()) {
//     std::cout << "去重: " << path.size() << " -> " << cleaned.size() << " 点"
//               << std::endl;
//   }
//   return cleaned;
// }
// // 预处理路径：去重 + 降采样
// std::vector<Point> TarjOpt::preprocessPath(const std::vector<Point> &path,
//                                            int downsample_step) {
//   // 1. 先去重
//   auto cleaned = removeDuplicates(path);
//   // 2. 再降采样
//   auto downsampled = downsamplePath(cleaned, downsample_step);
//   return downsampled;
// }

// Eigen::Matrix3Xd TarjOpt::Path2inPoints(const std::vector<Point> &path) {
//   size_t pieceNum = path.size() - 1;

//   // 如果只有起点和终点，没有中间点
//   if (pieceNum <= 1) {
//     return Eigen::Matrix3Xd(); // 返回空矩阵
//   }

//   Eigen::Matrix3Xd waypoints(3, pieceNum - 1);

//   // 填充中间点（跳过起点和终点）
//   for (size_t i = 1; i < path.size() - 1; i++) {
//     waypoints.col(i - 1) << path[i].x, path[i].y, 0.0;
//   }

//   std::cout << "中间点数量: " << waypoints.cols() << std::endl;
//   return waypoints;
// }
// // 时间分配
// Eigen::VectorXd TarjOpt::TimeAllocation(const std::vector<Point> &path) {
//   size_t pieceNum = path.size() - 1;
//   Eigen::VectorXd times(pieceNum);

//   // 计算每段距离和总距离
//   double total_distance = 0.0;
//   std::vector<double> distances(pieceNum);
//   for (size_t i = 0; i < path.size() - 1; i++) {
//     double dx = path[i + 1].x - path[i].x;
//     double dy = path[i + 1].y - path[i].y;
//     distances[i] = std::sqrt(dx * dx + dy * dy);
//     total_distance += distances[i];
//   }

//   if (total_distance < 1e-6) {
//     // 防止除零
//     for (size_t i = 0; i < pieceNum; i++) {
//       times(i) = 0.5;
//     }
//     return times;
//   }

//   // 使用适中的速度
//   double avg_velocity = 0.8; // m/s
//   double total_time = total_distance / avg_velocity;

//   // 按距离比例分配时间
//   for (size_t i = 0; i < pieceNum; i++) {
//     times(i) = (distances[i] / total_distance) * total_time;
//     times(i) = std::max(times(i), 0.1); // 最小0.1秒
//     times(i) = std::min(times(i), 2.0); // 最大2.0秒
//   }

//   return times;
// }
// bool TarjOpt::optimization(int downsample_step) {
//   if (path_.size() < 2) {
//     std::cerr << "错误：路径点数量不足" << std::endl;
//     return false;
//   }

//   std::cout << "\n=== MINCO优化开始 ===" << std::endl;
//   std::cout << "原始路径点数量: " << path_.size() << std::endl;

//   // 预处理路径：去重 + 降采样
//   std::vector<Point> processed_path = preprocessPath(path_, downsample_step);

//   if (processed_path.size() < 2) {
//     std::cerr << "错误：处理后路径点数量不足" << std::endl;
//     return false;
//   }

//   std::cout << "处理后路径点数量: " << processed_path.size() << std::endl;
//   std::cout << "起点: (" << processed_path.front().x << ", "
//             << processed_path.front().y << ")" << std::endl;
//   std::cout << "终点: (" << processed_path.back().x << ", "
//             << processed_path.back().y << ")" << std::endl;

//   int pieceNum = processed_path.size() - 1;
//   std::cout << "轨迹段数: " << pieceNum << std::endl;

//   // 边界条件
//   Eigen::Matrix<double, 3, 2> head, tail;
//   head << processed_path.front().x, processed_path.front().y, // 位置
//       0.0, 0.0,                                               // 速度
//       0.0, 0.0;                                               // 加速度

//   tail << processed_path.back().x, processed_path.back().y, // 位置
//       0.0, 0.0,                                             // 速度
//       0.0, 0.0;                                             // 加速度

//   // 设置优化器
//   minco_opt.setConditions(head, tail, pieceNum);

//   // 获取中间点和时间
//   auto inps = Path2inPoints(processed_path);
//   auto time = TimeAllocation(processed_path);

//   // 验证维度匹配
//   if (inps.cols() != pieceNum - 1 && pieceNum > 1) {
//     std::cerr << "错误：中间点数量不匹配！期望: " << pieceNum - 1
//               << ", 实际: " << inps.cols() << std::endl;
//     return false;
//   }

//   if (time.size() != pieceNum) {
//     std::cerr << "错误：时间向量大小不匹配！期望: " << pieceNum
//               << ", 实际: " << time.size() << std::endl;
//     return false;
//   }

//   // 打印前5个中间点
//   std::cout << "前5个中间点: ";
//   for (int i = 0; i < std::min(5, (int)inps.cols()); i++) {
//     std::cout << "(" << inps(0, i) << "," << inps(1, i) << ") ";
//   }
//   std::cout << std::endl;

//   // 执行优化
//   minco_opt.setParameters(inps, time);

//   // 获取优化结果
//   minco_opt.getTrajectory(opt_trajectory);

//   if (opt_trajectory.getPieceNum() == 0) {
//     std::cerr << "错误：轨迹生成失败" << std::endl;
//     return false;
//   }

//   std::cout << "优化成功！轨迹段数: " << opt_trajectory.getPieceNum()
//             << std::endl;
//   std::cout << "总时长: " << opt_trajectory.getTotalDuration() << " 秒"
//             << std::endl;

//   this->is_opt = true;
//   return true;
// }

// std::optional<std::vector<Point>> TarjOpt::get_traj() {
//   if (!this->is_opt) {
//     std::cerr << "警告：未优化，请先调用 optimization()" << std::endl;
//     return std::nullopt;
//   }

//   std::vector<Point> points_2d;
//   Eigen::Matrix3Xd positions = opt_trajectory.getPositions();

//   if (positions.cols() == 0) {
//     std::cerr << "警告：轨迹没有路径点" << std::endl;
//     return std::nullopt;
//   }

//   points_2d.reserve(positions.cols());
//   for (int i = 0; i < positions.cols(); i++) {
//     points_2d.emplace_back(positions(0, i), positions(1, i));
//   }

//   std::cout << "提取到 " << points_2d.size() << " 个路径点" << std::endl;
//   return points_2d;
// }

// // 获取轨迹总时长
// double TarjOpt::getDuration() {
//   if (is_opt) {
//     return opt_trajectory.getTotalDuration();
//   }
//   return 0.0;
// }

// } // namespace planner