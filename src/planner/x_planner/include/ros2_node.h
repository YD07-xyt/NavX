#pragma once
#include "plan_env/map.h"
#include "plan_env/sdf_map.h"
#include <Eigen/src/Core/Matrix.h>
#include <geometry_msgs/msg/detail/pose_stamped__struct.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <memory>
#include <nav_msgs/msg/detail/odometry__struct.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/timer.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
namespace ros2 {
class XPlannerROS2 {
public:
  XPlannerROS2(rclcpp::Node::SharedPtr node, const MapConfig map_config);

private:
  Eigen::Vector3d goal;

private:
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<planner::Map> map_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_gridmap_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_ESDF_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
      pub_gradESDF_;

  rclcpp::TimerBase::SharedPtr occ_timer_;
  rclcpp::TimerBase::SharedPtr esdf_timer_;
  rclcpp::TimerBase::SharedPtr vis_timer_;

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  void goalCloudCallback(const geometry_msgs::msg::PoseStamped::SharedPtr &goal);
  void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr &msg);
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr &msg);
  void publish_gridmap();
  void publish_ESDF();
  void publish_ESDFGrad();
};
} // namespace ros2