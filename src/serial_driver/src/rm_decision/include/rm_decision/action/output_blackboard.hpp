#pragma once
#include "rm_decision/config.hpp"
#include "rm_decision/api.hpp"
#include "rm_decision/log.hpp"
#include <behaviortree_cpp/action_node.h>
#include <behaviortree_cpp/basic_types.h>
#include <optional>
namespace bt {
class Outputblackboard {
public:
    explicit Outputblackboard(BT::Blackboard::Ptr tree_blackboard): tree_blackboard_(tree_blackboard) {};
    auto get_data() -> std::pair<std::optional<Decision2Game>, std::optional<Decision2Nav>> {
        return {get_game_data(), get_nav_data()};
    }

private:
    template<typename T>
    std::optional<T> get_blackboard_value(const std::string& key) const {
        T value {};
        if (!tree_blackboard_->get(key, value)) {
            return std::nullopt;
        }
        return value;
    }
    auto get_game_data() -> std::optional<Decision2Game> {
        auto sentry_opt = get_blackboard_value<bt::SentryModel>("sentry_model");
        if (!sentry_opt) return std::nullopt;
        return Decision2Game {sentry_opt.value()};
    }

    auto get_nav_data() -> std::optional<Decision2Nav> {
        auto nav_opt = get_blackboard_value<bt::Point>("goal");
        if (!nav_opt) return std::nullopt;
        return Decision2Nav {nav_opt.value()};
    }

private:
    BT::Blackboard::Ptr tree_blackboard_;
    std::optional<Decision2Game> to_game_;
    std::optional<Decision2Nav> to_nav_;
};
}