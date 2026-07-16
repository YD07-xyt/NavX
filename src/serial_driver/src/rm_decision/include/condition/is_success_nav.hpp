#pragma once
#include <behaviortree_cpp/basic_types.h>
#include<behaviortree_cpp/condition_node.h>
#include <optional>
#include<spdlog/spdlog.h>
#include"../config.hpp"
namespace bt {

class IsSuccessNav : public BT::ConditionNode{
public:
      IsSuccessNav(const std::string &name, const BT::NodeConfig &config)
      : BT::ConditionNode(name, config) {}

  // 必须的静态方法：声明输入输出端口
  static BT::PortsList providedPorts() {
    return {
        BT::InputPort<int>("current_nav_state")
    };
  }

  BT::NodeStatus tick() {
    int state;
    if(!getInput<int>("current_nav_state",state)){
        return BT::NodeStatus::FAILURE;
    };
    if(Int2Nav2State(state)!=std::nullopt){
        nav2_state_=Int2Nav2State(state).value();
    }else{
        spdlog::warn("get nav2_state failed in IsSuccessNav");
        return BT::NodeStatus::FAILURE;
    }
    if(nav2_state_==Nav2State::succeeded){
        return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::FAILURE;
  };
private:
  Nav2State nav2_state_=Nav2State::idle;

};

}