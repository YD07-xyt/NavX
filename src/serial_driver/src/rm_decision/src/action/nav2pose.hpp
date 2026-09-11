#pragma once
#include "rm_decision/api.hpp"
#include "rm_decision/config.hpp"
#include "rm_decision/log.hpp"
#include "tools/logger.hpp"
#include <behaviortree_cpp/action_node.h>
#include <cstddef>
#include <optional>
#include <spdlog/spdlog.h>
#include <string>
namespace bt {
class Nav2Pose: public BT::StatefulActionNode {
public:
    Nav2Pose(const std::string& name, const BT::NodeConfig& config):
        BT::StatefulActionNode(name, config),
        navigation_goal_(0.0, 0.0, 0.0) {};

    // this function is invoked once at the beginning.
    BT::NodeStatus onStart() override {
        auto goal = getInput<bt::Point>("goal");
        nav_state_ = getInput<NavState>("nav_state");
        if (!goal) {
            logger::warn(logger, "[Nav2Pose]goal is not set");
            return BT::NodeStatus::FAILURE;
        }
        if (!nav_state_) {
            logger::warn(logger, "[Nav2Pose]nav_state is not set");
            return BT::NodeStatus::FAILURE;
        }
        setOutput<bt::Point>("goal2nav", goal.value());
        return BT::NodeStatus::RUNNING;
    };
    // If onStart() returned RUNNING, we will keep calling
    // this method until it return something different from RUNNING
    BT::NodeStatus onRunning() override {
        if (!nav_state_) {
            logger::warn(logger, "[Nav2Pose]nav_state is not set");
            return BT::NodeStatus::FAILURE;
        }
        switch (nav_state_.value()) {
            case NavState::IDLE:
                return BT::NodeStatus::RUNNING;
            case NavState::RUNNING:
                return BT::NodeStatus::RUNNING;
            case NavState::SUCCEEDED:
                return BT::NodeStatus::SUCCESS;
            case NavState::FAILURE:
                return BT::NodeStatus::FAILURE;
            default:
                logger::error(logger, "[Nav2Pose]unexpected nav_state: {}", nav_state_.value());
                return BT::NodeStatus::FAILURE;
        }
    }
    // callback to execute if the action was aborted by another node
    void onHalted() override {
        logger::info(logger, "[Nav2Pose] Halted by external interrupt.");

        // 重置内部持有的状态，确保下次 onStart 时逻辑干净
        nav_state_ = {};
    };

    static BT::PortsList providedPorts() {
        const char* description = "goal send to nav.";
        return {
            BT::InputPort<bt::Point>("goal", "目标点输入"),
            BT::InputPort<NavState>("nav_state", "导航状态"),
            BT::OutputPort<bt::Point>("goal2nav", "输出给导航的目标点")};
    };

private:
    double timeout_ = 0.8;
    std::chrono::steady_clock::time_point last_pub_time_;
    bt::Point navigation_goal_;
    BT::Expected<NavState> nav_state_;
};
} // namespace bt