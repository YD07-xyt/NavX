#pragma once
#include "config.hpp"
#include <behaviortree_cpp/action_node.h>
#include <spdlog/spdlog.h>
namespace bt {
class Chase : BT::StatefulActionNode {
public:
  Chase(const std::string &name, const BT::NodeConfiguration &config)
      : BT::StatefulActionNode(name, config) {}
  static BT::PortsList providedPorts() {
    return {BT::InputPort<bt::Point>("current_pose"),
            BT::InputPort<bt::Point>("enemy_pose")};
  }

  // 启动时读取参数并初始化
  BT::NodeStatus onStart() override {
    if (!getInput<bt::Point>("current_pose", current_pose_)) {
      spdlog::error("[Chase]read current_pose failed");
    }
    if (!getInput<bt::Point>("enemy_pose", enemy_pose_)) {
      spdlog::error("[Chase]read enemy_pose failed");
    }
    return BT::NodeStatus::RUNNING; // 进入 running 状态
  }

  // 每个 tick 被调用
  BT::NodeStatus onRunning() override { return BT::NodeStatus::RUNNING; }

  // 若被中断，清理（本例无资源）
  void onHalted() override {}

private:
  bt::Point current_pose_;
  bt::Point enemy_pose_;
};

} // namespace bt