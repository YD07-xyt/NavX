#pragma once
#include <behaviortree_cpp/action_node.h>
#include <string>

namespace bt {

class SetBlackboardBool : public BT::SyncActionNode {
public:
  SetBlackboardBool(const std::string &name, const BT::NodeConfig &config)
      : BT::SyncActionNode(name, config) {}

  static BT::PortsList providedPorts() {
    return {BT::InputPort<std::string>("key", "要设置的键名"),
            BT::InputPort<bool>("value", "要设置的值")};
  }

  BT::NodeStatus tick() override {
    auto key = getInput<std::string>("key");
    auto val = getInput<bool>("value");

    if (!key || !val)
      return BT::NodeStatus::FAILURE;

    auto root_bb = config().blackboard->rootBlackboard();
    if (!root_bb) {
      // 如果获取失败，回退到当前黑板
      config().blackboard->set(*key, *val);
    } else {
      root_bb->set(*key, *val);
    }

    return BT::NodeStatus::SUCCESS;
  }
};

} // namespace bt