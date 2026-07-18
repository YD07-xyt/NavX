#pragma once
#include <behaviortree_cpp/basic_types.h>
#include <behaviortree_cpp/condition_node.h>
#include <spdlog/spdlog.h>

namespace bt {

class IsGameTimeExceeded : public BT::ConditionNode {
public:
  IsGameTimeExceeded(const std::string &name, const BT::NodeConfig &config)
      : BT::ConditionNode(name, config) {}

  static BT::PortsList providedPorts() {
    return {BT::InputPort<int>("game_time", "当前游戏时间（秒）"),
            BT::InputPort<int>("threshold", "阈值（秒）")};
  }

  BT::NodeStatus tick() override {
    //spdlog::info("[IsGameTimeExceeded] blackboard ptr = {}", fmt::ptr(config().blackboard.get()));
    if (!getInput<int>("game_time", game_time_)) {
      spdlog::warn(
          "[IsGameTimeExceeded] failed read game_time from input port");
      // 尝试直接从黑板读取
      int blackboard_time;
      if (config().blackboard->get<int>("game_time", blackboard_time)) {
        spdlog::warn("But blackboard has game_time = {}", blackboard_time);
      } else {
        spdlog::warn("Blackboard does NOT have game_time key");
      }
      return BT::NodeStatus::FAILURE;
    }


    // threshold 来自 XML 属性，可以用 getInput
    if (!getInput<int>("threshold", threshold_)) {
      spdlog::warn("IsGameTimeExceeded: 读取 threshold 失败");
      return BT::NodeStatus::FAILURE;
    }

    //> 为大于阈值触发
    if (game_time_ >= threshold_) { 
      spdlog::debug("[IsGameTimeExceeded] 时间>阈值");
      return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::FAILURE;
  }

private:
  int threshold_ ;
  int game_time_ = 0;
};

} // namespace bt