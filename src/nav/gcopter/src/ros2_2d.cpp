#include "ros2/ros2_2d.hpp"
#include "config.hpp"
#include "controller/omni_lmpc.hpp"
#include "fsm/fsm.hpp"
#include "utils/plotter.hpp"
#include <Eigen/src/Core/Matrix.h>
#include <chrono>
#include <cstddef>
#include <geometry_msgs/msg/detail/twist__struct.hpp>
#include <memory>
#include <nav_msgs/msg/detail/path__struct.hpp>
#include <optional>
#include <spdlog/spdlog.h>
#include <string>
#include <tf2/LinearMath/Matrix3x3.hpp>
#include <tf2/LinearMath/Quaternion.hpp>

namespace planner {
GlobalPlanner2d::GlobalPlanner2d(rclcpp::Node::SharedPtr nh_)
    : config(nh_), nh(nh_), mapInitialized(false), visualizer(nh_),
      fsm_(config.fsm_config), plotter_(), omni_lmpc_(config.lmpc_param) {

  grid_map_ = std::make_shared<grid_map::GridMap>();

  grid_map_->init(config.map_size, config.map_size, config.resolution);

  start_time_ = std::chrono::steady_clock::now();

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

  // 每 0.1 秒执行一次
  planner_timer_ =
      nh->create_wall_timer(std::chrono::milliseconds(33), // 时间间隔参数
                            [&]() { planner_callback(); }  // 回调函数
      );
  controller_timer_ =
      nh->create_wall_timer(std::chrono::milliseconds(33), // 时间间隔参数
                            [&]() { controller_callback(); } // 回调函数
      );
}
void GlobalPlanner2d::controller_callback() {
  if (!current_XYTheta.has_value()) {
    // spdlog::warn("[controller_callback] no current_XYtheta");
    return;
  }
  if (!trajectory_.isInitialized()) {
    // spdlog::warn("轨迹未初始化，等待规划...");
    return;
  }
  // 距离目标检查（优先）
  if ((current_XYTheta->head<2>() - goal_pose->head<2>()).norm() < 0.05) {
    geometry_msgs::msg::Twist stop;
    cmd_vel_pub_->publish(stop);
    spdlog::debug("到达目标点，停止控制");
    return;
  }

  // 更新状态并求解（参考游标由 MPC 内部按机器人实际进度跟踪，
  //    不再依赖墙钟时间，避免落后参考时反复切角、偏差累积）
  omni_lmpc_.update_current_pose(*current_XYTheta);
  auto predicted = omni_lmpc_.slover(trajectory_);
  if (predicted.empty()) {
    spdlog::warn("MPC 求解失败");
    return;
  }

  // 发布控制指令
  Eigen::Vector3d u_cmd = omni_lmpc_.u_k;
  geometry_msgs::msg::Twist twist;
  twist.linear.x = u_cmd.x();
  twist.linear.y = u_cmd.y();
  twist.angular.z = u_cmd.z();
  cmd_vel_pub_->publish(twist);
  
}

void GlobalPlanner2d::planner_callback() { GlobalPlanner2d::plan_omni(); }
void GlobalPlanner2d::plan_omni() {
  if (this->goal_pose == std::nullopt) {
    spdlog::debug("no goal");
    return;
  }
  if (this->current_pose == std::nullopt) {
    spdlog::debug("no current_pose");
    return;
  }

  auto result = fsm_.plan(goal_pose.value(), current_pose.value(), grid_map_);
  if (result) {
    auto [astar_path, opt] = result.value();
    visualizer.PubGlobalPath(astar_path);
    std::vector<Eigen::Vector2d> opt_path = opt.sampleTrajectory(0.1);
    visualizer.PubOptPath(opt_path);

    trajectory_ = opt.getOptimizedTrajectory();
    visualizer.PubWayPoints(trajectory_);
    std::vector<Eigen::Vector2d> dense_path = opt.sampleTrajectory(0.02);
    visualizer.PubTrajectory(trajectory_, dense_path);
  }
}
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
};

void GlobalPlanner2d::mapCallBack(
    const sensor_msgs::msg::PointCloud2::SharedPtr &msg) {
  auto voxel_num = grid_map_->getVoxelNum();
  grid_map::RowMatrixXi occupancy =
      grid_map::RowMatrixXi::Zero(voxel_num.x(), voxel_num.y());

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

    Eigen::Vector2d obstacle_world =
        Eigen::Vector2d(fdata[cur + 0], fdata[cur + 1]);
    Eigen::Vector2i grid_index;
    grid_map_->posToIndex(obstacle_world, grid_index);

    if (grid_index.x() >= 0 && grid_index.x() < voxel_num.x() &&
        grid_index.y() >= 0 && grid_index.y() < voxel_num.y()) {
      occupancy(grid_index.x(), grid_index.y()) = 1; // 标记为障碍物
      pc.emplace_back(Eigen::Vector3d(fdata[cur + 0], fdata[cur + 1], 0));
    }
  }
  grid_map_->setMap(occupancy);
  mapInitialized = true;

  visualizer.visualizeMap(pc);
}
void GlobalPlanner2d::targetCallBack(
    const geometry_msgs::msg::PoseStamped::SharedPtr &msg) {
  spdlog::info("Received target pose with position ({:2f},{:2f},{:2f})",
               msg->pose.position.x, msg->pose.position.y,
               msg->pose.position.z);
  if (mapInitialized) {

    const Eigen::Vector3d goal(msg->pose.position.x, msg->pose.position.y, 0.0);

    goal_pose = goal;

    spdlog::info("get goal");

  } else {
    spdlog::warn("map no init");
  }
  return;
}

} // namespace planner