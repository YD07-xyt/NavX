#pragma once
#include "../config.hpp"


//planner
#include"../st_opt/perception_tool/grid_map.hpp"
#include"../st_opt/frontend_tool/astar.hpp"
#include"../st_opt/backend_tool/traj_opt.hpp"

//map

#include "map/grid_map.h"


//controller
#include "controller/dwa_planner.h"
#include "controller/differential_mpc.h"
#include "controller/traj_tracker.hpp"
//gcopter
#include "gcopter/firi.hpp"
#include "gcopter/flatness.hpp"
#include "gcopter/gcopter.hpp"
#include "gcopter/sfc_gen.hpp"
#include "gcopter/trajectory.hpp"
#include "gcopter/voxel_map.hpp"

//3rd
#include <Eigen/Core>
//log
#include <Eigen/src/Core/Matrix.h>
#include <spdlog/spdlog.h>
//ros2 
#include "misc/visualizer.hpp"

#include <geometry_msgs/msg/detail/twist__struct.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/detail/path__struct.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/time.hpp>
#include <rclcpp/timer.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

//std
#include <optional>
#include <cstddef>
#include <chrono>
#include <cmath>
#include <iostream>
#include <memory>
#include <random>
#include <string>
#include <vector>

namespace planner {

class GlobalPlanner2d {
private:
  Config config;
  rclcpp::Node::SharedPtr nh;
  // planner::KinodynamicAstar astar_planner_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr mapSub;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr targetSub;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr OdomSub;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr global_path_pub_;
  rclcpp::TimerBase::SharedPtr planner_timer_;
  rclcpp::TimerBase::SharedPtr controller_timer_;
  std::optional<Eigen::Vector3d> current_pose = std::nullopt;
  std::optional<Eigen::Vector3d> goal_pose = std::nullopt;
  bool mapInitialized;
  Visualizer visualizer;
  
  //============================//
  std::shared_ptr<grid_map::GridMap> grid_map_;

  //===========================//

  //contorller
  std::optional<Eigen::Vector3d> current_XYTheta=std::nullopt;
  std::shared_ptr<Mpc> mpc_;
  TrajectoryTracker traj_tracker_;
  std::shared_ptr<DWAPlanner> dwa_planner_; 
  DwaConfig dwa_config_;

public:
  GlobalPlanner2d(rclcpp::Node::SharedPtr nh_);
  void mapCallBack(const sensor_msgs::msg::PointCloud2::SharedPtr &msg);
  void odomCallBack(const nav_msgs::msg::Odometry::SharedPtr &msg);
  void targetCallBack(const geometry_msgs::msg::PoseStamped::SharedPtr &msg);
  void planner_callback();
  void controller_callback();
  void plan_omni();
  void plan_omni_corr();
  
  std::vector<Eigen::Vector3d> replan(std::vector<Eigen::Vector3d> route) {
    // 检查初始路径是否有效
    while (!(route.back().x() == goal_pose->x() &&
             route.back().y() == goal_pose->y())) {
      route.clear();
      // // 可添加最大重规划次数防止无限循环                         
    }
    spdlog::info("replan success");
    return route;
  }
};
} // namespace planner