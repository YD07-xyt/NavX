#pragma once
#include <action_msgs/msg/detail/goal_status_array__struct.hpp>
#include <action_msgs/msg/goal_status_array.hpp>
#include <geometry_msgs/msg/detail/twist__struct.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
namespace test {
class TestNode {
public:
  TestNode(rclcpp::Node::SharedPtr node) : node_(node) {
    goal_state_pub_ =
        node_->create_publisher<action_msgs::msg::GoalStatusArray>("", 10);
    goal_sub_ = node_->create_subscription<geometry_msgs::msg::Twist>(
        "", 10, [this](geometry_msgs::msg::Twist::SharedPtr &data) {
          GoalSubCallback(data);
        });
  }

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<action_msgs::msg::GoalStatusArray>::SharedPtr
      goal_state_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr goal_sub_;
  void GoalSubCallback(geometry_msgs::msg::Twist::SharedPtr &data) {}
};
} // namespace test