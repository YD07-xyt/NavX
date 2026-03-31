#pragma once

#ifndef FSM_CONDITION_H
#define FSM_CONDITION_H
#include "state.h"
namespace decision::fsm {

//rm层
inline bool is_go_home(StateSet current_state) {
  auto [curent_game, current_nav, current_robot] = current_state.current();
  if (current_robot.current_HP < 100 ||
      current_robot.projectile_allowance < 50) {
    return true;
  } else {
    return false;
  }
}


inline bool is_start_game(StateSet current_state){
    auto current_game = current_state.game();
    if(current_game==GameState::Running){
        return true;
    }else{
        return false;
    }
}

inline bool is_to_HitOutPost(StateSet current_state){
    //auto 
}


// nav层
inline bool is_nav_running(StateSet current_state){
    auto current_nav = current_state.nav();
    if(current_nav==NavState::Running){
        return false;
    }else{
        return true;
    }
}



} // namespace decision::fsm
#endif