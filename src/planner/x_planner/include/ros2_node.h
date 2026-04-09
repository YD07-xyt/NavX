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
  XPlannerROS2(rclcpp::Node::SharedPtr node, const MapConfig map_config)
      : node_(node) {
    map_ = std::make_shared<planner::Map>(
        map_config, [this]() { publish_ESDF(); },
        [this]() { publish_gridmap(); });
    pub_ESDF_ =
        node_->create_publisher<sensor_msgs::msg::PointCloud2>("esdf", 10);
    pub_gridmap_ =
        node_->create_publisher<sensor_msgs::msg::PointCloud2>("grid_map", 10);
    pub_gradESDF_ =
        node_->create_publisher<visualization_msgs::msg::MarkerArray>(
            "pub_gradESDF_", 10);

    goal_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
        "cloud_map", 10, [this](const geometry_msgs::msg::PoseStamped goal) {
          goalCloudCallback(goal);
        });
    cloud_sub_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
        "cloud_map", 10,
        [this](const sensor_msgs::msg::PointCloud2::SharedPtr &msg) {
          pointCloudCallback(msg);
        });
    odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
        "cloud_map", 10, [this](const nav_msgs::msg::Odometry::SharedPtr &msg) {
          odomCallback(msg);
        });
        
    occ_timer_ = node_->create_wall_timer(
        std::chrono::milliseconds(50),
        std::bind(&SDFmap::updateOccupancyCallback, map_->sdf_map_.get()));

    esdf_timer_ = node_->create_wall_timer(
        std::chrono::milliseconds(50),
        std::bind(&SDFmap::updateESDFCallback, map_->sdf_map_.get()));

    vis_timer_ = node_->create_wall_timer(
        std::chrono::milliseconds(500),
        std::bind(&SDFmap::visCallback, map_->sdf_map_.get()));
  };

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
  void goalCloudCallback(const geometry_msgs::msg::PoseStamped goal);
  void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr &msg);
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr &msg);
  void publish_gridmap();
  void publish_ESDF();
  void publish_ESDFGrad();
};
} // namespace ros2