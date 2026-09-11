#pragma once
#include "rm_decision/config.hpp"
#include "rm_decision/api.hpp"
#include <behaviortree_cpp/action_node.h>
#include <behaviortree_cpp/basic_types.h>
#include <Eigen/Core>
namespace bt {
class Intputblackboard {
public:
  explicit Intputblackboard(BT::Blackboard::Ptr tree_blackboard)
      : tree_blackboard_(tree_blackboard){};
  auto set_data(Game2Decision &game_data,Nav2Decision &nav_data)->void{
    set_nav_data(nav_data);
    set_game_data(game_data);
  }
private:
  template <typename T>
  void set_blackboard_value(const std::string &key, const T& value) {
      tree_blackboard_->set(key, value);
  }
  void set_game_data(Game2Decision &game_data) {
    set_blackboard_value<int>("current_hp", game_data.current_hp);
    set_blackboard_value<bool>("is_game_start", game_data.is_game_start);
    set_blackboard_value<bool>("ours_fort_occ_state", game_data.ours_fort_occ_state);
    set_blackboard_value<int>("game_time", game_data.game_time);
    set_blackboard_value<int>("current_enemy_outpost_hp", game_data.current_enemy_outpost_hp);
   
    set_blackboard_value<bt::Point>("current_pose", game_data.current_pose);
    set_blackboard_value<bt::Point>("chase_pose", game_data.chase_pose);
    set_blackboard_value<int>("current_projectile_allowance", game_data.projectile_allowance);
  }
  void set_nav_data(Nav2Decision &nav_data) {
    set_blackboard_value<NavState>("nav_state", nav_data.nav_state);
  }

private:
  BT::Blackboard::Ptr tree_blackboard_;
};
} // namespace bt