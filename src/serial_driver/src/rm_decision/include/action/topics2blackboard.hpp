#pragma once
#include "../config.hpp"
#include <behaviortree_cpp/action_node.h>
#include <behaviortree_cpp/basic_types.h>
#include <Eigen/Core>
#include <spdlog/spdlog.h>
namespace bt {
class Topics2Blackboard {
public:
  Topics2Blackboard(BT::Blackboard::Ptr tree_blackboard)
      : tree_blackboard_(tree_blackboard){};

public:
  struct GameData {
    bool is_game_start;
    int current_hp;
    int projectile_allowance;
    //bool is_enemy_outpost_destroyed;
    int current_enemy_outpost_hp;
    int game_time;
    bt::Point current_pose;
    bt::Point chase_pose;
  };
  void TopicSetCallback(GameData &game_data) {
    tree_blackboard_->set<int>("current_hp", game_data.current_hp);
    tree_blackboard_->set<bool>("is_game_start", game_data.is_game_start);
    // spdlog::info("[Topics2Blackboard] blackboard ptr = {}", fmt::ptr(tree_blackboard_.get()));
    // spdlog::info("[Topics2Blackboard] set game_time = {}", game_data.game_time);

    tree_blackboard_->set<int>("game_time", game_data.game_time);
    tree_blackboard_->set<int>("current_enemy_outpost_hp", game_data.current_enemy_outpost_hp);
    tree_blackboard_->set<bt::Point>("current_pose",
                               game_data.current_pose);
    tree_blackboard_->set<bt::Point>("chase_pose",
                               game_data.chase_pose);
    tree_blackboard_->set<int>("current_projectile_allowance",
                               game_data.projectile_allowance);
  }

private:
  BT::Blackboard::Ptr tree_blackboard_;
};
} // namespace bt