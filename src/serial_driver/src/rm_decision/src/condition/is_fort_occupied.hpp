#pragma once
#include <behaviortree_cpp/basic_types.h>
#include <behaviortree_cpp/condition_node.h>
#include <spdlog/spdlog.h>
namespace bt {
class IsFortOccupied : public BT::ConditionNode {
public:
  IsFortOccupied(const std::string &name, const BT::NodeConfig &config)
      : BT::ConditionNode(name, config) {}
      // 必须的静态方法：声明输入输出端口
  static BT::PortsList providedPorts() {
    return {BT::InputPort<bool>("ours_fort_occ_state")};
  }

  BT::NodeStatus tick() {
    if(!getInput<bool>("ours_fort_occ_state",ours_fort_occ_state_)){
        return BT::NodeStatus::FAILURE;
    }
    spdlog::debug("[IsFortOccupied]ours_fort_occ_state : {}",ours_fort_occ_state_);
    if(ours_fort_occ_state_==true){
      spdlog::info("[IsFortOccupied] ours fort is occupied");
        return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::FAILURE;
  };
private:
  bool ours_fort_occ_state_;
};

} // namespace bt
