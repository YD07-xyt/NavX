#pragma once
#include"type.h"
namespace decision{
    class FSM{
        public:
            FSM();
            void decision(Nav2State nav2_state,int is_game,int current_hp,int projectile_allowance);
            void init_nav_state(Nav2State nav2_state);
        private:
            void Patrol();
            void gohome();
            void go2goal();
            void wait(float wait_time);
        private:
            RobotState robot_state_ = RobotState::move;
            Nav2State nav2_state_ = Nav2State::unkown;
            RobotTask robot_task_ = RobotTask::no_start;
        private:
            void choose_task();
            void switch_robot_state(int is_game,int current_hp,int projectile_allowance);
            
    };
}