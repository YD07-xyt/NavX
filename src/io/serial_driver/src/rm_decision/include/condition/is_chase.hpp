#pragma once

#include "config.hpp"
#include <behaviortree_cpp/basic_types.h>
#include <behaviortree_cpp/condition_node.h>
#include <spdlog/spdlog.h>

namespace bt {
class IsChase : public BT::ConditionNode {
public:
  IsChase(const std::string &name, const BT::NodeConfig &config)
      : BT::ConditionNode(name, config){};
  // 必须的静态方法：声明输入输出端口
  static BT::PortsList providedPorts() {
    return {BT::InputPort<bt::Point>("current_game_state"),
    BT::InputPort<bt::Point>("enemy_pose")};
  };

  BT::NodeStatus tick(){
    if(!getInput<bt::Point>("current_pose",current_pose_)){
        return BT::NodeStatus::FAILURE;
    }
    if(!getInput<bt::Point>("enemy_pose",enemy_pose_)){
        spdlog::debug("[IsChase]no chase enemy");
        return BT::NodeStatus::FAILURE;
    }else{
        spdlog::info("[IsChase]find chase enemy");
        return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::FAILURE;
  };
private:
  auto rating_chase(bt::Point current_pose,bt::Point enemy_pose)->float{
    //TODO:
    return 0.0;
  }
  float score_;
  bt::Point enemy_pose_;
  bt::Point current_pose_;
};
} // namespace bt