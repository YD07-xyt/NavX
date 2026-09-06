#pragma once

#include "action/intput_blackboard.hpp"
#include "action/output_blackboard.hpp"
#include "api.hpp"
#include "config.hpp"
#include <behaviortree_cpp/actions/sleep_node.h>
#include <behaviortree_cpp/behavior_tree.h>
#include <behaviortree_cpp/blackboard.h>
#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/json_export.h>
#include <behaviortree_cpp/loggers/groot2_publisher.h>
#include <behaviortree_cpp/xml_parsing.h>

#include <memory>
#include <optional>
#include <utility>

namespace bt {
class RmDecision {
public:
    explicit RmDecision(bt::DecisionConfig decision_config);

    auto tree_tick(
        Game2Decision& game_data,
        Nav2Decision& nav_data,
        std::optional<std::chrono::system_clock::duration> timeout
    ) -> void;
    auto get_data() -> std::pair<std::optional<Decision2Game>, std::optional<Decision2Nav>>;

private:
    void setting_tree_custom_node();

private:
    bt::DecisionConfig config_;
    std::shared_ptr<Intputblackboard> intput_blackboard_;
    std::shared_ptr<Outputblackboard> output_blackboard_;

private:
    BT::Tree tree_;
    BT::Blackboard::Ptr tree_blackboard_;
    std::shared_ptr<BT::Groot2Publisher> groot2publisher_ptr_;
    BT::BehaviorTreeFactory factory_;
};
} // namespace bt
