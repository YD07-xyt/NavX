#pragma once

#include <behaviortree_cpp/basic_types.h>
#include <behaviortree_cpp/condition_node.h>
#include <spdlog/spdlog.h>
namespace bt {
class IsRobotHealth :  public BT::ConditionNode {
public:
  IsRobotHealth(const std::string &name, const BT::NodeConfig &config)
      : BT::ConditionNode(name, config) {}

  // 必须的静态方法：声明输入输出端口
  static BT::PortsList providedPorts() {
    return {BT::InputPort<int>("current_projectile_allowance"), // 当前弹药量
            BT::InputPort<int>("min_projectile_allowance"),
            BT::InputPort<int>("current_hp"),
            BT::InputPort<int>("min_hp")}; // 阈值
  }

  BT::NodeStatus tick() {
    if (!getInput<int>("current_projectile_allowance",
                       current_projectile_allowance_) ||
        !getInput<int>("min_projectile_allowance", min_projectile_allowance_) ||
        !getInput<int>("current_hp", current_hp_) ||
        !getInput<int>("min_hp", min_hp_)) {
      spdlog::warn("[IsOursBaseHealth] failed read param");
      return BT::NodeStatus::FAILURE;
    }
    if (current_hp_>= min_hp_&&current_projectile_allowance_>min_projectile_allowance_) {
      spdlog::info("[IsRobotHealth]robot is health");
      return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::FAILURE;
  };

private:
  // no Destroyed ==true
  int current_hp_;
  int min_hp_;
  int current_projectile_allowance_;
  int min_projectile_allowance_;
};
} // namespace bt