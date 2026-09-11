#pragma once
#include "config.hpp"
#include <behaviortree_cpp/bt_factory.h>
namespace bt {
struct Decision2Nav {
    Point goal_point;
};
enum NavState {
    IDLE, //一般未开始
    FAILURE,
    SUCCEEDED,
    RUNNING,
};
struct Nav2Decision {
    NavState nav_state;
};
struct Game2Decision {
    bool is_game_start;//比赛是否开始
    int current_hp;//哨兵的血量
    int projectile_allowance;//所并
    //bool is_enemy_outpost_destroyed;
    int current_enemy_outpost_hp;
    int game_time;
    bt::Point current_pose;
    bt::Point chase_pose;
    bool ours_fort_occ_state;
};
enum SentryModel{
    ATTACK = 1,
    DEFENSE = 2,
    MOVE = 3,
};
struct Decision2Game {
    SentryModel sentry_model;
};
inline auto register_enum(BT::BehaviorTreeFactory& factory)->void{
    factory.registerScriptingEnums<SentryModel>();
    factory.registerScriptingEnums<NavState>();
}
}