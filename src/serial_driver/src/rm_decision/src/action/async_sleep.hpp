#pragma once
#include <behaviortree_cpp/action_node.h>
#include <chrono>

namespace bt {

class AsyncSleep : public BT::StatefulActionNode {
public:
    AsyncSleep(const std::string &name, const BT::NodeConfig &config)
        : BT::StatefulActionNode(name, config) {}

    BT::NodeStatus onStart() override {
        auto msec = getInput<double>("msec");
        if (!msec) {
            return BT::NodeStatus::FAILURE;
        }
        start_time_ = std::chrono::steady_clock::now();
        duration_ = std::chrono::milliseconds(static_cast<int>(msec.value()));
        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override {
        auto now = std::chrono::steady_clock::now();
        if (now - start_time_ >= duration_) {
            return BT::NodeStatus::SUCCESS;
        }
        return BT::NodeStatus::RUNNING;
    }

    void onHalted() override {}

    static BT::PortsList providedPorts() {
        return {BT::InputPort<double>("msec")};
    }

private:
    std::chrono::steady_clock::time_point start_time_;
    std::chrono::milliseconds duration_;
};

} // namespace bt