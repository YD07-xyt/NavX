#pragma once
#include <Eigen/Geometry>
#include <action_msgs/msg/goal_status_array.hpp>
#include <memory>
#include <nav2_costmap_2d/nav2_costmap_2d/costmap_2d.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include<nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include "../config.hpp"
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <spdlog/spdlog.h>
#include <tf2_eigen/tf2_eigen/tf2_eigen.hpp>

namespace ros2 {
class Ros2Node {
public:
  Ros2Node(){};
  Ros2Node(std::optional<bt::NodeWithParam> &node_with_param)
      : node_(node_with_param.value().node) {
    this->config_ = std::make_unique<bt::DecisionConfig>(
        node_with_param.value().decision_config);
    this->pub_nav2_state_ =
        std::make_unique<bt::Nav2State>(bt::Nav2State::idle);
    this->action_client_ =
        rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
            node_, "navigate_to_pose");
    this->goal_pub_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>(
        config_->pub_goal_topic_name, 10);
    this->nav2_status_sub_ =
        node_->create_subscription<action_msgs::msg::GoalStatusArray>(
            config_->nav2_state_topic_name, 10,
            [this](const action_msgs::msg::GoalStatusArray &msg) {
              this->Nav2StateCallback(msg);
            });
    // this->odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
    //     config_->odom_sub_topic, 10,
    //     [this](const nav_msgs::msg::Odometry &msg) {
    //       this->OdomCallback(msg);
    //     });
  }

  auto pub_goal(std::string &pub_model, bt::Point &goal_point) -> bool {
    if (pub_model == "action") {
      navigation_goal_.pose.pose.position.x = goal_point.x;
      navigation_goal_.pose.pose.position.y = goal_point.y;
      navigation_goal_.pose.pose.orientation.w = 1.0;
      navigation_goal_.pose.pose.orientation.x = 0.0;
      navigation_goal_.pose.pose.orientation.y = 0.0;
      navigation_goal_.pose.pose.orientation.z = 0.0;
      navigation_goal_.pose.header.frame_id = "map";
      navigation_goal_.pose.header.stamp = node_->now();
      auto future_goal_handle =
          action_client_->async_send_goal(navigation_goal_);
      if (rclcpp::spin_until_future_complete(
              node_, future_goal_handle,
              std::chrono::milliseconds(config_->send_goal_timeout)) !=
          rclcpp::FutureReturnCode::SUCCESS) {
        spdlog::error("send goal failed");
        return false;
      }
      action_goal_handle_ = future_goal_handle.get();
      if (!action_goal_handle_) {
        spdlog::error("goal handle is null");
        return false;
      }
      spdlog::info("[action]Published goal: ({}, {},{} rad)", goal_point.x,
                   goal_point.y, goal_point.yaw);
      return true;
    } else if (pub_model == "publisher") {

      auto msg = geometry_msgs::msg::PoseStamped();

      // 设置时间戳和坐标系
      msg.header.stamp = node_->now();
      msg.header.frame_id = this->config_->map_tf_name; //!!!

      // 设置位置
      msg.pose.position.x = goal_point.x;
      msg.pose.position.y = goal_point.y;
      msg.pose.position.z = 0.0;

      // 设置方向（四元数）
      msg.pose.orientation.x = 0.0;
      msg.pose.orientation.y = 0.0;
      msg.pose.orientation.z = 0.0;
      msg.pose.orientation.w = 1.0;

      goal_pub_->publish(msg);
      spdlog::info("[publisher] Published goal: ({}, {}, {} rad)", goal_point.x,
                   goal_point.y, goal_point.yaw);
      return true;
    } else {
      spdlog::warn("model is error in pub_goal");
      return false;
    }
    return false;
  }
  auto cancel_action_pub_goal() -> bool {
    if (action_goal_handle_) {
      auto cancel_future =
          action_client_->async_cancel_goal(action_goal_handle_);
      if (rclcpp::spin_until_future_complete(node_, cancel_future) !=
          rclcpp::FutureReturnCode::SUCCESS) {
        spdlog::error("cancel goal failed");
        return false;
      }
      spdlog::info("goal canceled");
      return true;
    }
    return false;
  };
  auto get_nav2_state(std::string &nav2_state_model)
      -> std::optional<bt::Nav2State> {
    if (nav2_state_model == "action") {
      SwitchActionNav2State(action_goal_handle_);
      if (action_goal_handle_->get_status() ==
          action_msgs::msg::GoalStatus::STATUS_SUCCEEDED) {
        return bt::Nav2State::succeeded;
      } else if (action_goal_handle_->get_status() ==
                     action_msgs::msg::GoalStatus::STATUS_ABORTED ||
                 action_goal_handle_->get_status() ==
                     action_msgs::msg::GoalStatus::STATUS_CANCELED) {
        return bt::Nav2State::aborted;
      } else {
        return bt::Nav2State::running;
      }
    } else if (nav2_state_model == "publisher") {
      return *pub_nav2_state_;
    } else {
      spdlog::warn("model is error in pub_goal");
      return std::nullopt;
    }
    return std::nullopt;
  };
  //auto get_curent_pose() -> bt::Point { return *this->current_pose_; }

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr
      action_client_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
  rclcpp::Subscription<action_msgs::msg::GoalStatusArray>::SharedPtr
      nav2_status_sub_;
  nav2_msgs::action::NavigateToPose::Goal navigation_goal_;
  // goal handle
  rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr
      action_goal_handle_;
  //定位
  //rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

private:
  std::unique_ptr<bt::DecisionConfig> config_;
  std::unique_ptr<bt::Nav2State> pub_nav2_state_;
  //std::unique_ptr<bt::Point> current_pose_;
  //std::shared_ptr<nav2_costmap_2d::Costmap2D> costmap_;
  
private:
  //void costmapCallback(const nav_msgs::msg::OccupancyGrid& msg);

  // void OdomCallback(const nav_msgs::msg::Odometry &msg) {
  //   Eigen::Quaterniond q;
  //   tf2::fromMsg(msg.pose.pose.orientation, q); // 需要 tf2_eigen

  //   // 转为旋转矩阵再取欧拉角 (ZYX 顺序得到 yaw, pitch, roll)
  //   Eigen::Vector3d euler = q.toRotationMatrix().eulerAngles(2, 1, 0);
  //   double yaw =
  //       euler(0); // 或直接使用 eulerAngles 的顺序，注意与常用 RPY 定义一致
  //   double pitch = euler(1);
  //   double roll = euler(2);
  //   current_pose_->x = msg.pose.pose.position.x;
  //   current_pose_->x = msg.pose.pose.position.y;
  //   current_pose_->yaw = yaw;
  // }
  void Nav2StateCallback(const action_msgs::msg::GoalStatusArray &msg) {
    if (msg.status_list.empty()) {
      spdlog::warn("GoalStatusArray msg is empty");
      return;
    }
    // 获取最新的状态
    auto &last_status = msg.status_list.back();

    if (last_status.status == action_msgs::msg::GoalStatus::STATUS_SUCCEEDED) {
      if (*pub_nav2_state_ != bt::Nav2State::succeeded) {
        *pub_nav2_state_ = bt::Nav2State::succeeded;
        spdlog::debug("导航成功！");
      }
    } else if (last_status.status ==
               action_msgs::msg::GoalStatus::STATUS_ABORTED) {
      if (*pub_nav2_state_ != bt::Nav2State::aborted) {
        *pub_nav2_state_ = bt::Nav2State::aborted;
        spdlog::debug("导航失败/终止");
      }
    } else if (last_status.status ==
               action_msgs::msg::GoalStatus::STATUS_EXECUTING) {
      if (*pub_nav2_state_ != bt::Nav2State::running) {
        *pub_nav2_state_ = bt::Nav2State::running;
        spdlog::debug("导航执行中");
      }
    }
  }

  void SwitchActionNav2State(
      rclcpp_action::ClientGoalHandle<
          nav2_msgs::action::NavigateToPose>::SharedPtr goal_handle) {
    switch (goal_handle->get_status()) {
    case action_msgs::msg::GoalStatus::STATUS_UNKNOWN:
      spdlog::info("goal status: STATUS_UNKNOWN");
      break;
    case action_msgs::msg::GoalStatus::STATUS_ACCEPTED:
      spdlog::info("goal status: STATUS_ACCEPTED");
      break;
    case action_msgs::msg::GoalStatus::STATUS_EXECUTING:
      spdlog::info("goal status: STATUS_EXECUTING");
      break;
    case action_msgs::msg::GoalStatus::STATUS_CANCELING:
      spdlog::info("goal status: STATUS_CANCELING");
      break;
    case action_msgs::msg::GoalStatus::STATUS_SUCCEEDED:
      spdlog::info("goal status: STATUS_SUCCEEDED");
      break;
    case action_msgs::msg::GoalStatus::STATUS_CANCELED:
      spdlog::info("goal status: STATUS_CANCELED");
      break;
    case action_msgs::msg::GoalStatus::STATUS_ABORTED:
      spdlog::info("goal status: STATUS_ABORTED");
      break;
    default:
      spdlog::info("goal status: ERROR CODE");
      break;
    }
  }
};
} // namespace ros2