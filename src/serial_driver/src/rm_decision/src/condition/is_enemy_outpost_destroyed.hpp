#pragma once
#include <behaviortree_cpp/basic_types.h>
#include <behaviortree_cpp/condition_node.h>
#include <spdlog/spdlog.h>
namespace bt {
class IsEnemyOutpostDestroyed : public BT::ConditionNode {
public:
  IsEnemyOutpostDestroyed(const std::string &name, const BT::NodeConfig &config)
      : BT::ConditionNode(name, config) {}
      // 必须的静态方法：声明输入输出端口
  static BT::PortsList providedPorts() {
    return {BT::InputPort<int>("enemy_outpost_hp"),
            BT::InputPort<int>("min_enemy_outpost_hp")};
  }

  BT::NodeStatus tick() override {
    if(!getInput<int>("enemy_outpost_hp",enemy_outpost_hp_)||
        !getInput<int>("min_enemy_outpost_hp",min_enemy_outpost_hp_)){
        return BT::NodeStatus::FAILURE;
    }
    spdlog::debug("[IsEnemyOutpostDestroyed]current_enemy_outpost_hp : {}",enemy_outpost_hp_);
    if(enemy_outpost_hp_<min_enemy_outpost_hp_){
      spdlog::debug("[IsEnemyOutpostDestroyed] enemy_outpost is destroyed");
        return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::FAILURE;
  };
private:
  // no Destroyed ==true
  int enemy_outpost_hp_;
  int min_enemy_outpost_hp_;
};

} // namespace bt
