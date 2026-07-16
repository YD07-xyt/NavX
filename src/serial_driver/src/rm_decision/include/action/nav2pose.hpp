#pragma once
#include "../config.hpp"
#include <behaviortree_cpp/action_node.h>
#include <cstddef>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <spdlog/spdlog.h>
#include <string>
#include "../ros2/ros2_node.hpp"
namespace bt {
class Nav2Pose : public BT::StatefulActionNode {
public:
  Nav2Pose(const std::string &name, const BT::NodeConfig &config,
           std::optional<bt::NodeWithParam> &node_with_param)
      : BT::StatefulActionNode(name, config), node_(node_with_param->node),
        navigation_goal_(0.0, 0.0, 0.0) {
    ros2_nav2_node_.emplace(node_with_param);
  };

  // this function is invoked once at the beginning.
  BT::NodeStatus onStart() override {
    auto goal = getInput<bt::Point>("goal");
    auto pub_model = getInput<std::string>("pub_model");
    if (!goal) {
      spdlog::warn("[Nav2Pose]goal is not set");
      return BT::NodeStatus::FAILURE;
    }
    if (!pub_model) {
      spdlog::warn("[Nav2Pose]pub_model is not set");
      return BT::NodeStatus::FAILURE;
    }
    navigation_goal_ = goal.value();
    nav2_state_ = std::nullopt;  // 重置状态，防止旧值干扰
    auto pub_temp =
        ros2_nav2_node_.value().pub_goal(pub_model.value(), goal.value());
    if (!pub_temp) {
      spdlog::warn("[Nav2Pose]pub goal failed");
      return BT::NodeStatus::FAILURE;
    }
    return BT::NodeStatus::RUNNING;
  };
  // If onStart() returned RUNNING, we will keep calling
  // this method until it return something different from RUNNING
  BT::NodeStatus onRunning() override {
    auto pub_model = getInput<std::string>("pub_model");
    if (!pub_model) {
      spdlog::warn("[Nav2Pose]pub_model is not set");
      return BT::NodeStatus::FAILURE;
    }

    // 1. 检查目标是否改变（如果改变则重新发布）
    auto new_goal = getInput<bt::Point>("goal");
    if (new_goal) {
      if (std::abs(new_goal.value().x - navigation_goal_.x) > 1e-6 ||
          std::abs(new_goal.value().y - navigation_goal_.y) > 1e-6 ||
          std::abs(new_goal.value().yaw - navigation_goal_.yaw) > 1e-6) {
        spdlog::info("goal updated");
        auto pub_bool = ros2_nav2_node_.value().pub_goal(pub_model.value(),
                                                         new_goal.value());
        if (!pub_bool) {
          spdlog::warn("[Nav2Pose] pub goal failed");
          return BT::NodeStatus::FAILURE;
        }
        navigation_goal_ = new_goal.value();
      }
    }

    // 2. 【核心修正】不管目标是否改变，都要读取当前导航状态
    nav2_state_ = ros2_nav2_node_.value().get_nav2_state(pub_model.value());
    if (!nav2_state_.has_value()) {
      spdlog::warn("[Nav2Pose] get nav2_state is empty");
      return BT::NodeStatus::FAILURE;
    }

    // 3. 根据状态返回结果
    if (nav2_state_.value() == bt::Nav2State::succeeded) {
      return BT::NodeStatus::SUCCESS;
    } else if (nav2_state_.value() == bt::Nav2State::aborted) {
      return BT::NodeStatus::FAILURE;
    } else {
      return BT::NodeStatus::RUNNING;
    }
  }
  // callback to execute if the action was aborted by another node
  void onHalted() override {
    auto succeess_bool = ros2_nav2_node_.value().cancel_action_pub_goal();
    if (succeess_bool) {
      spdlog::error("[onHalted] goal halted failed");
    }
  };

  static BT::PortsList providedPorts() {
    const char *description = "goal send to navigator.";
    return {BT::InputPort<bt::Point>("goal", description),
            BT::InputPort<std::string>("pub_model", "发布模式标识")};
  };

private:
  rclcpp::Node::SharedPtr node_;
  std::optional<ros2::Ros2Node> ros2_nav2_node_;
  bt::Point navigation_goal_;
  std::optional<bt::Nav2State> nav2_state_;
};
} // namespace bt