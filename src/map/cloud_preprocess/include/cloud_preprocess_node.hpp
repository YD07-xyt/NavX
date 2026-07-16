#pragma once

/**
 * @file cloud_preprocess_node.hpp
 * @brief ROS2 接口层：负责话题收发，调用 cloud_preprocess 核心库。
 *
 * 输入（单一）：
 *   - sensor_msgs/msg/PointCloud2，话题 input_cloud_topic
 *
 * 发布：
 *   - obstacles_topic   : sensor_msgs/msg/PointCloud2  （障碍物点云）
 *   - ground_topic      : sensor_msgs/msg/PointCloud2  （过滤后的地面点云）
 *   - inflated_topic    : sensor_msgs/msg/PointCloud2  （膨胀点云地图，可选）
 */

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl_conversions/pcl_conversions.h>

#include <string>
#include <vector>

#include "cloud_preprocess.hpp"

namespace cloud_preprocess {

/// @brief 点云预处理 ROS2 节点
class CloudPreprocessNode : public rclcpp::Node {
 public:
  explicit CloudPreprocessNode(
      const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

 private:
  // 回调：点云
  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

  // 通用处理+发布
  void processAndPublish(const CloudPtr& cloud,
                         const std_msgs::msg::Header& header);

  // 累计点云维护
  void accumulate(const CloudPtr& input, const rclcpp::Time& stamp);
  void pruneAccumulation(const rclcpp::Time& now);
  CloudPtr getAccumulated() const;

  // 从参数服务器加载算法参数到 core_
  void loadParams();

  // 核心算法库
  CloudPreprocess core_;

  // 订阅 / 发布
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_cloud_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_obstacles_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_ground_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_inflated_;

  // 话题名（由参数 input_*_topic / *_topic 配置）
  std::string topic_input_cloud_ = "~/input_cloud";
  std::string topic_obstacles_ = "~/obstacles";
  std::string topic_ground_ = "~/ground";
  std::string topic_inflated_ = "~/inflated";

  // 累计点云参数（累计输入点云后再统一处理）
  bool enable_accumulate_ = false;
  double accumulate_range_ = 5.0;    // 机器人周围保留半径 (m)
  double accumulate_lifetime_ = 2.0; // 点存活时间 (s)

  // 累计点云存储（绝对坐标 + 时间戳）
  std::vector<PointT> acc_pts_;
  std::vector<rclcpp::Time> acc_stamps_;
};

}  // namespace cloud_preprocess
