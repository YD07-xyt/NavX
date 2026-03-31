#include "include/action.h"
#include "include/condition.h"
#include"include/fsm.h"
#include "include/state.h"
#include <cstdio>
//#include "tool/glog.h"
//TODO: 未实现
void get_serial_data();
//TOD: 
void SerialData2State(){
    // if(game_state==4){
    //     game =  decision::fsm::GameState::Running;
    //     state_set.change_GameState(game);
    // }else{
    //     game =  decision::fsm::GameState::NotRunning;
    //     state_set.change_GameState(game);
    // }
};
int main(){
    int robot_HP;
    int projectile_allowance;
    int game_state;

    decision::fsm::StateSet state_set;
    decision::fsm::GameState game;

    get_serial_data();
    SerialData2State();

    // TODO: 如何处理nav 导航时的state 冲突 (eg: running时是否发新点)
    // 目前必须要求running完一个点
    // TODO: 是否考虑nav running 的限制时间( 这要解决decision与导航模块决策的冲突)
    // 比赛是否开始
    if(decision::fsm::is_start_game(state_set)){
        //开始
        if(decision::fsm::is_go_home(state_set)){
            //需要回家补给
            //TODO: 
            decision::fsm::pubHome();
        }else{
            if(decision::fsm::is_to_HitOutPost(state_set)){
                decision::fsm::pubHitOutPost();
            }else{
                
            }

        }

    }else{
        //TODO: log --> glog
        printf("game is not start");
    }
    return 0; 
}