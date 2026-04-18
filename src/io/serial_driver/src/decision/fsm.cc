#include "fsm.h"
#include <iostream>

namespace decision {
    void FSM::decision(Nav2State nav2_state,int is_game,int current_hp,int projectile_allowance){
        this->init_nav_state(nav2_state);

        if(!is_game){
            std::cout<<"[warn]game is not start"<<std::endl;
            return;
        }
        switch_robot_state(is_game,current_hp,projectile_allowance);
        choose_task();
        switch (robot_task_){
            case RobotTask::no_start:
                std::cout<<"[info]robot_task_ no_start"<<std::endl;
                break;
            case RobotTask::Patrol:
                this->Patrol();
                break;
            case RobotTask::go2goal:
                this->go2goal();
                break;
            case RobotTask::run2home:
                this->gohome();
                break;
        }
    }
    void FSM::Patrol(){
        std::cout<<"[info]robot_task_ Patrol"<<std::endl;
    
    }
    void FSM::init_nav_state(Nav2State nav2_state){
        this->nav2_state_ = nav2_state;
    }
    void FSM::choose_task(){
        if(this->robot_state_ == RobotState::supply){
            robot_task_ = RobotTask::run2home;
        }else {
            robot_task_ = RobotTask::Patrol;
        }
    }
    void FSM::switch_robot_state(int is_game,int current_hp,int projectile_allowance){
        if(current_hp<=100||projectile_allowance<=50){
            robot_state_ = RobotState::supply;
        }else{
            robot_state_ = RobotState::move;
        }
    }
}
