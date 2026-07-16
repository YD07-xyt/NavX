#pragma once
#include "../config.hpp"
#include <behaviortree_cpp/action_node.h>
#include <spdlog/spdlog.h>
#include <string>

namespace bt {
class SwitchNavModel : public BT::SyncActionNode {
public:
  SwitchNavModel(const std::string &name, const BT::NodeConfig &config)
      : BT::SyncActionNode(name, config) {}

  BT::NodeStatus tick() override {
    std::string nav_model;
    if (!getInput<std::string>("model", nav_model))
      return BT::NodeStatus::FAILURE;
    if (nav_model != "follow" && nav_model != "spinning"&& nav_model!="downhill")
      return BT::NodeStatus::FAILURE;

    // 获取根黑板（主树的黑板）
    auto root_bb = config().blackboard->rootBlackboard();
    if (!root_bb) {
      // 如果获取失败，回退到当前黑板
      config().blackboard->set("nav_model", nav_model);
    } else {
      root_bb->set("nav_model", nav_model);
    }
    spdlog::info("[SwitchNavModel] set nav_model to {} (root)", nav_model);
    return BT::NodeStatus::SUCCESS;
  }

  static BT::PortsList providedPorts() {
    return {
        BT::InputPort<std::string>("model",
                                   "切换导航模式(跟随云台or小陀螺模式)"),
    };
  }
};
} // namespace bt