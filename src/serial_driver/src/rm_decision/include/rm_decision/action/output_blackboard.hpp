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
        return {to_game_, to_nav_};
    }

private:
    template<typename T>
    std::optional<T> get_blackboard_value(const std::string& key) const {
        return tree_blackboard_->get<T>(key);
    }
    auto get_game_data() -> std::optional<Decision2Game> {
        auto sentry_opt = get_blackboard_value<bt::SentryModel>("sentry_model");
        if (sentry_opt) { // 或者 if (sentry_opt.has_value())
            to_game_.value().sentry_model = sentry_opt.value();
            return to_game_;
        } else {
            // 处理缺失情况：记录错误、使用默认值、或返回 FAILURE
            logger::error(logger, "sentry_model not found on blackboard");
            // 可设置默认值或直接返回失败状态
            return std::nullopt;
        }
        return std::nullopt;
    }
    auto get_nav_data() -> std::optional<Decision2Nav> {
        auto nav_opt = get_blackboard_value<bt::Point>("goal");
        if (nav_opt) {
            to_nav_.value().goal_point = nav_opt.value();
            return to_nav_;
        } else {
            logger::error(logger, "goal not found on blackboard");
            return std::nullopt;
        }
        return std::nullopt;
    }

private:
    BT::Blackboard::Ptr tree_blackboard_;
    std::optional<Decision2Game> to_game_;
    std::optional<Decision2Nav> to_nav_;
};
}