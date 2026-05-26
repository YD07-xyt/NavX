#include "ros2/ros2_2d.hpp"
#include "config.hpp"
#include "controller/dwa_planner.h"
#include <Eigen/src/Core/Matrix.h>
#include <cstddef>
#include <geometry_msgs/msg/detail/twist__struct.hpp>
#include <memory>
#include <nav_msgs/msg/detail/path__struct.hpp>
#include <optional>
#include <spdlog/spdlog.h>
#include <tf2/LinearMath/Matrix3x3.hpp>
#include <tf2/LinearMath/Quaternion.hpp>

namespace planner {
GlobalPlanner2d::GlobalPlanner2d(rclcpp::Node::SharedPtr nh_)
    : config(nh_), nh(nh_), mapInitialized(false), visualizer(nh_),
      dwa_config_(), traj_tracker_(0.1, 10) {
  grid_map_ = std::make_shared<grid_map::GridMap>();
  const double map_size = 20.0;
  const double resolution = 0.1;
  grid_map_->init(map_size, map_size, resolution);

  dwa_planner_ = std::make_shared<DWAPlanner>();
  dwa_planner_->setLimits(dwa_config_.max_speed, dwa_config_.min_speed,
                          dwa_config_.max_yawrate, dwa_config_.max_accel,
                          dwa_config_.max_dyawrate);
  dwa_planner_->setObsRange(dwa_config_.range);
  dwa_planner_->setPredict(dwa_config_.dt, dwa_config_.predict_time);
  dwa_planner_->setResolution(dwa_config_.v_resolution,
                              dwa_config_.yawrate_resolution);
  dwa_planner_->setRobotRadius(dwa_config_.robot_radius);
  dwa_planner_->setSamples(dwa_config_.velocity_samples,
                           dwa_config_.yawrate_samples);
  dwa_planner_->setWeights(dwa_config_.weights_to_goal,
                           dwa_config_.weights_obstacle,
                           dwa_config_.weights_speed);

  mpc_ = std::make_shared<Mpc>();
  mpc_->setWeights(config.mpc_weights);

  mapSub = nh->create_subscription<sensor_msgs::msg::PointCloud2>(
      config.mapTopic, rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        GlobalPlanner2d::mapCallBack(msg);
      });
  OdomSub = nh->create_subscription<nav_msgs::msg::Odometry>(
      config.odomTopic, rclcpp::QoS(10),
      [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
        GlobalPlanner2d::odomCallBack(msg);
      });
  targetSub = nh->create_subscription<geometry_msgs::msg::PoseStamped>(
      config.targetTopic, rclcpp::QoS(10),
      [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        targetCallBack(msg);
      });
  cmd_vel_pub_ =
      nh->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel_chassis", 10);
  global_path_pub_ =
      nh->create_publisher<nav_msgs::msg::Path>("global_path", 10);
  // 每 0.1 秒执行一次
  planner_timer_ =
      nh->create_wall_timer(std::chrono::milliseconds(100), // 时间间隔参数
                            [&]() { planner_callback(); }   // 回调函数
      );
  controller_timer_ =
      nh->create_wall_timer(std::chrono::milliseconds(100), // 时间间隔参数
                            [&]() { controller_callback(); } // 回调函数
      );
}
void GlobalPlanner2d::controller_callback() {
  if(current_pose.has_value()&&goal_pose.has_value()){
    if ((current_pose->x() == goal_pose->x()) &&
        (current_pose->y() == goal_pose->y())) {
      spdlog::info("success to goal");
    }
  }
  if (!current_XYTheta.has_value()) {
    spdlog::warn("current_XYTheta no value");
  }
  if(current_XYTheta.has_value()){
    if(traj_tracker_.is_full_trajectory()){
      auto v_w = traj_tracker_.update(current_XYTheta.value());
      geometry_msgs::msg::Twist cmd_pub_data;
      cmd_pub_data.angular.z = v_w.y();
      cmd_pub_data.linear.x = v_w.x();
      spdlog::info("v:{},w:{}", v_w.x(), v_w.y());
      this->cmd_vel_pub_->publish(cmd_pub_data);
    }
  }
}
void GlobalPlanner2d::planner_callback() { GlobalPlanner2d::plan_omni(); }
void GlobalPlanner2d::plan_omni() {
  if (this->goal_pose == std::nullopt) {
    // spdlog::info("no goal");
  }
  if (this->current_pose == std::nullopt) {
    // spdlog::info("no current_pose");
  }
  if (this->goal_pose != std::nullopt && this->current_pose != std::nullopt) {
    // Start timing
    auto start_time = std::chrono::high_resolution_clock::now();

    // ==================== 3. Define Start and Goal ====================
    const Eigen::Vector2d start(current_pose->x(), current_pose->y());
    const Eigen::Vector2d goal(goal_pose->x(), goal_pose->y());

    // ==================== 4. A* Global Search ====================
    path_planning::AStar astar(*grid_map_, 0.3); // 0.1m safety threshold
    auto astar_traj =
        astar.planWithPostProcessing(start, goal, 5000); // 5s timeout

    if (astar_traj.optimized_path.empty()) {
      std::cerr << "A* planning failed!" << std::endl;
      return;
    }
    // goal_pose=std::nullopt;
    // PubPath(astar_traj.optimized_path);  // 替代原来的
    // PubPath(astar_traj.optimized_path)
    //  PubPath(astar_traj.optimized_path);
    visualizer.PubGlobalPath(astar_traj.optimized_path);

    // std::cout << "=== A* Planning Results ===" << std::endl;
    // std::cout << "Optimized path points: " <<
    // astar_traj.optimized_path.size()
    //           << std::endl;
    // std::cout << "Total length: " << astar_traj.total_length << " m"
    //           << std::endl;
    // std::cout << "Total time: " << astar_traj.total_time << " s" <<
    // std::endl;

    // End timing
    auto end_time = std::chrono::high_resolution_clock::now();
    // ==================== 5. Trajectory Optimization ====================
    TrajOpt::TrajectoryParams params;
    params.piece_len = astar_traj.total_length / astar_traj.total_time;
    params.total_time = astar_traj.total_time;
    params.total_len = astar_traj.total_length;

    TrajOpt::TrajectoryOptimizer optimizer(grid_map_, astar_traj.optimized_path,
                                           params);

    if (!optimizer.plan()) {
      std::cerr << "Trajectory optimization failed!" << std::endl;
      return;
    }

    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(
        end_time - start_time);

    // ==================== 6. Evaluate Optimization Results
    // ====================
    auto metrics = optimizer.evaluateTrajectory();
    // std::cout << "\n=== Optimization Metrics ===" << std::endl;
    // std::cout << "Max velocity: " << metrics.max_velocity << " m/s" <<
    // std::endl; std::cout << "Min clearance: " << metrics.min_clearance << "
    // m" << std::endl; std::cout << "Path deviation: " <<
    // metrics.path_deviation << " m" << std::endl; std::cout << "Trajectory
    // energy: " << metrics.trajectory_energy << std::endl; std::cout <<
    // "Optimization time: " << duration.count() << " ms" << std::endl;

    // ==================== 7. Visualize Optimized Trajectory
    // ====================
    std::vector<Eigen::Vector2d> opt_path =
        optimizer.sampleTrajectory(0.1); // Sample every 0.1s
    visualizer.PubOptPath(opt_path);
    spdlog::info("path points:{}", opt_path.size());

    //==================================================//
    //======================MPC========================//
    //==================================================//
    traj_tracker_.setReferenceTrajectory(opt_path);
  }
}
// Eigen::Vector2d GlobalPlanner2d::getLocalGoal(double x, double y,
//   const std::vector<Eigen::Vector2d>& path,double lookahead)
// {
//     if (path.empty()) return {x, y}; // fallback

//     // 1. 找最近点索引
//     int idx_min = 0;
//     double min_dist = std::numeric_limits<double>::max();
//     for (int i = 0; i < (int)path.size(); ++i) {
//         double dx = path[i].x() - x;
//         double dy = path[i].y() - y;
//         double d = std::sqrt(dx*dx + dy*dy);
//         if (d < min_dist) {
//             min_dist = d;
//             idx_min = i;
//         }
//     }

//     // 2. 从最近点向前累计距离，寻找前瞻点
//     double cum_dist = 0.0;
//     int target_idx = idx_min;
//     for (int i = idx_min + 1; i < (int)path.size(); ++i) {
//         double dx = path[i].x() - path[i-1].x();
//         double dy = path[i].y() - path[i-1].y();
//         cum_dist += std::sqrt(dx*dx + dy*dy);
//         target_idx = i;
//         if (cum_dist >= lookahead) break;
//     }

//     return path[target_idx];
// }
void GlobalPlanner2d::odomCallBack(
    const nav_msgs::msg::Odometry::SharedPtr &msg) {
  if (!current_pose.has_value()) {
    current_pose = Eigen::Vector3d::Zero();
  }
  if (!current_XYTheta.has_value()) {
    current_XYTheta = Eigen::Vector3d::Zero();
  }
  const auto &quat = msg->pose.pose.orientation;
  tf2::Quaternion tf_quat(quat.x, quat.y, quat.z, quat.w);
  double roll, pitch, yaw;
  tf2::Matrix3x3(tf_quat).getRPY(roll, pitch, yaw);

  current_pose->x() = msg->pose.pose.position.x;
  current_pose->y() = msg->pose.pose.position.y;
  current_pose->z() = msg->pose.pose.position.z;
  current_XYTheta->x() = msg->pose.pose.position.x;
  current_XYTheta->y() = msg->pose.pose.position.y;
  current_XYTheta->z() = yaw;
  // spdlog::info("callback current_XYTheta: x:{},y:{},yaw:{}",
  //     current_XYTheta->x(),current_XYTheta->y(),current_XYTheta->z());
  // spdlog::info("callback
  // current_plose_:x:{},y:{}",current_pose->x(),current_pose->y());
  //  spdlog::info("Received current pose with position ({},{},{})",
  //               msg->pose.pose.position.x, msg->pose.pose.position.y,
  //               msg->pose.pose.position.z);
};

void GlobalPlanner2d::mapCallBack(
    const sensor_msgs::msg::PointCloud2::SharedPtr &msg) {
  // RCLCPP_INFO(nh->get_logger(), "Received map point cloud with %zu points",
  //             msg->data.size() / msg->point_step);
  grid_map::RowMatrixXi occupancy = grid_map::RowMatrixXi::Zero(200, 200);
  auto voxel_num = grid_map_->getVoxelNum();
  std::vector<Eigen::Vector3d> pc;
  size_t cur = 0;
  const size_t total = msg->data.size() / msg->point_step;
  float *fdata = (float *)(&msg->data[0]);
  for (size_t i = 0; i < total; i++) {
    cur = msg->point_step / sizeof(float) * i;
    if (std::isnan(fdata[cur + 0]) || std::isinf(fdata[cur + 0]) ||
        std::isnan(fdata[cur + 1]) || std::isinf(fdata[cur + 1]) ||
        std::isnan(fdata[cur + 2]) || std::isinf(fdata[cur + 2])) {
      spdlog::warn("map continue");
      continue;
    }
    //====================================//
    Eigen::Vector2d obstacle_world =
        Eigen::Vector2d(fdata[cur + 0], fdata[cur + 1]);
    Eigen::Vector2i grid_index;
    grid_map_->posToIndex(obstacle_world, grid_index);

    if (grid_index.x() >= 0 && grid_index.x() < voxel_num.x() &&
        grid_index.y() >= 0 && grid_index.y() < voxel_num.y()) {
      occupancy(grid_index.x(), grid_index.y()) = 1; // 标记为障碍物
      pc.emplace_back(Eigen::Vector3d(fdata[cur + 0], fdata[cur + 1], 0));
    }
    //===============================================//
  }
  grid_map_->setMap(occupancy);
  mapInitialized = true;

  visualizer.visualizeMap(pc);
}
void GlobalPlanner2d::targetCallBack(
    const geometry_msgs::msg::PoseStamped::SharedPtr &msg) {
  spdlog::info("Received target pose with position ({},{},{})",
               msg->pose.position.x, msg->pose.position.y,
               msg->pose.position.z);
  if (mapInitialized) {

    const double zGoal =
        config.mapBound[4] + config.dilateRadius +
        fabs(msg->pose.orientation.z) *
            (config.mapBound[5] - config.mapBound[4] - 2 * config.dilateRadius);
    const Eigen::Vector3d goal(msg->pose.position.x, msg->pose.position.y, 0.0);

    goal_pose = goal;

    spdlog::info("get goal");

  } else {
    spdlog::warn("map no init");
  }
  return;
}

} // namespace planner