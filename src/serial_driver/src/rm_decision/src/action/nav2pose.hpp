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

   BT::NodeStatus onStart() override {
    auto goal = getInput<bt::Point>("goal");
    if (!goal) {
        logger::warn(logger, "[Nav2Pose]goal is not set");
        return BT::NodeStatus::FAILURE;
    }
    logger::info(logger, "[Nav2Pose]set goal:{},{},{}", goal->x, goal->y, goal->yaw);

    nav_state_ = getInput<NavState>("nav_state");
    if (!nav_state_) {
        logger::warn(logger, "[Nav2Pose]nav_state is not set");
        return BT::NodeStatus::FAILURE;
    }

    seen_active_ = false;   // 新目标，先认为还没激活

    setOutput<bt::Point>("goal2nav", goal.value());
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus onRunning() override {
    nav_state_ = getInput<NavState>("nav_state");
    if (!nav_state_) {
        logger::warn(logger, "[Nav2Pose]nav_state is not set");
        return BT::NodeStatus::FAILURE;
    }

    switch (nav_state_.value()) {
        case NavState::IDLE:
        case NavState::RUNNING:
            seen_active_ = true;
            return BT::NodeStatus::RUNNING;

        case NavState::SUCCEEDED:
            // 如果本次目标还没见过 IDLE/RUNNING，说明这是上一个目标的残留状态
            return seen_active_ ? BT::NodeStatus::SUCCESS : BT::NodeStatus::RUNNING;

        case NavState::FAILURE:
            return seen_active_ ? BT::NodeStatus::FAILURE : BT::NodeStatus::RUNNING;

        default:
            logger::error(logger, "[Nav2Pose]unexpected nav_state: {}", nav_state_.value());
            return BT::NodeStatus::FAILURE;
    }
}

void onHalted() override {
    logger::info(logger, "[Nav2Pose] Halted by external interrupt.");
    nav_state_ = {};
    seen_active_ = false;
}


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
    bool seen_active_;
};
} // namespace bt