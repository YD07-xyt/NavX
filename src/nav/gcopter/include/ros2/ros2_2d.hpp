#pragma once
#include "../config.hpp"


//planner

#include"nav.hpp"
//map


//controller
#include "controller/omni_lmpc.hpp"
//gcopter
#include "fsm/fsm.hpp"


//3rd
#include <Eigen/Core>
//log
#include <Eigen/src/Core/Matrix.h>
#include <spdlog/spdlog.h>
//ros2 
#include "misc/visualizer.hpp"
#include "utils/plotter.hpp"

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

  rclcpp::TimerBase::SharedPtr planner_timer_;
  rclcpp::TimerBase::SharedPtr controller_timer_;
  std::optional<Eigen::Vector3d> current_pose = std::nullopt;
  std::optional<Eigen::Vector3d> goal_pose = std::nullopt;
  std::optional<Eigen::Vector3d> current_XYTheta=std::nullopt;
  bool mapInitialized;
  Visualizer visualizer;
  
private:
  //地图
  std::shared_ptr<grid_map::GridMap> grid_map_;
  //重规划
  FSM fsm_;
  
  tools::Plotter plotter_;
private:
  //contorller
  controller::LMpc omni_lmpc_;
  double t_now_;
  SplineTrajectory::PPolyND<2> trajectory_;
  std::chrono::steady_clock::time_point start_time_;
public:
  GlobalPlanner2d(rclcpp::Node::SharedPtr nh_);
  void mapCallBack(const sensor_msgs::msg::PointCloud2::SharedPtr &msg);
  void odomCallBack(const nav_msgs::msg::Odometry::SharedPtr &msg);
  void targetCallBack(const geometry_msgs::msg::PoseStamped::SharedPtr &msg);
  void planner_callback();
  void controller_callback();
  void plan_omni();

};
} // namespace planner