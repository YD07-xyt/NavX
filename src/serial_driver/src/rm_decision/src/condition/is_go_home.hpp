#pragma once
#include <behaviortree_cpp/basic_types.h>
#include <behaviortree_cpp/condition_node.h>
#include <spdlog/spdlog.h>
namespace bt {
class IsGoHome : public BT::ConditionNode {
public:
  IsGoHome(const std::string &name, const BT::NodeConfig &config)
      : BT::ConditionNode(name, config) {}

  // 必须的静态方法：声明输入输出端口
  static BT::PortsList providedPorts() {
    return {BT::InputPort<int>("current_projectile_allowance"), // 当前弹药量
            BT::InputPort<int>("min_projectile_allowance"),
            BT::InputPort<int>("current_hp"),
            BT::InputPort<int>("min_hp")}; // 阈值
  }

  BT::NodeStatus tick() {
    // 从端口获取值（会自动关联黑板）
    if (!getInput<int>("current_projectile_allowance",
                       current_projectile_allowance_) ||
        !getInput<int>("min_projectile_allowance", min_projectile_allowance_) ||
        !getInput<int>("current_hp", current_hp_) ||
        !getInput<int>("min_hp", min_hp_)) {
      spdlog::warn("[IsGoHome] failed read param");
      return BT::NodeStatus::FAILURE;
    }
    spdlog::debug("[IsGoHome]current_hp:{}",current_hp_);
    if (current_projectile_allowance_ < min_projectile_allowance_ ||
        current_hp_ < min_hp_) {
      spdlog::info("[IsGoHome]need go home");
      return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::FAILURE;
  }
  
private:
    int current_projectile_allowance_, min_projectile_allowance_, min_hp_,
        current_hp_;
};
} // namespace bt