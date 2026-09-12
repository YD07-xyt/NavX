#pragma once

#include "rm_decision/log.hpp"
#include <behaviortree_cpp/basic_types.h>
#include <behaviortree_cpp/condition_node.h>


namespace bt {
class IsGameStart : public BT::ConditionNode {
public:
  IsGameStart(const std::string &name, const BT::NodeConfig &config)
      : BT::ConditionNode(name, config){};
  // 必须的静态方法：声明输入输出端口
  static BT::PortsList providedPorts() {
    return {BT::InputPort<bool>("current_game_state")};
  };

  BT::NodeStatus tick() override{

    if(!getInput<bool>("current_game_state",is_game_start_)){
        return BT::NodeStatus::FAILURE;
    }
    if(is_game_start_==true){
      //spdlog::info("game is start");
        return BT::NodeStatus::SUCCESS;
    }
    logger::info(logger,"game is not start");
    return BT::NodeStatus::FAILURE;
  };
private:
  bool is_game_start_=false;
};
} // namespace bt