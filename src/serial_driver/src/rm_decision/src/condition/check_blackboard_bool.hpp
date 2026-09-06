#pragma once
#include <behaviortree_cpp/condition_node.h>
#include <string>

namespace bt {

class CheckBlackboardBool : public BT::ConditionNode {
public:
    CheckBlackboardBool(const std::string& name, const BT::NodeConfig& config)
        : BT::ConditionNode(name, config) {}

    static BT::PortsList providedPorts() {
        return {
            BT::InputPort<std::string>("key", "黑板中的布尔键名"),
            BT::InputPort<bool>("value", "期望值")
        };
    }

    BT::NodeStatus tick() override {
        auto key = getInput<std::string>("key");
        auto expected = getInput<bool>("value");
        if (!key || !expected) return BT::NodeStatus::FAILURE;

        auto blackboard = config().blackboard;
        auto actual = blackboard->get<bool>(*key);
        if (actual == *expected) {
            return BT::NodeStatus::SUCCESS;
        }
        return BT::NodeStatus::FAILURE;
    }
};

} // namespace bt