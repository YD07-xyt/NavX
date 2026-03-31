#pragma once


#ifndef FSM_STATE_H
#define FSM_STATE_H

#include<tuple>

namespace decision::fsm {
    enum class GameState{
        Running,
        NotRunning,
    };
    enum class NavState{
        Idle,           //空闲
        Success,        //成功
        Failed,         //失败
        Running,        //运行
        //TODO: nav2中  等待取消  取消中  已取消 是否考虑
        Cancel,         //取消
    };
    enum SentryPose{
        Move,
        Attack,
        Defend,
    };
    struct RobotState{
        double current_HP;
        double projectile_allowance;
        //TODO: 暂时不用
        double outpost_HP;
        bool is_attacked;
        RobotState(){
            current_HP=400;
            projectile_allowance=300;
            //TODO
            outpost_HP=2000;
        }
    };
    class StateSet {
        public:
            StateSet(){
                this->game_=GameState::NotRunning;
                this->nav_= NavState::Idle;
                this->sentry_pose_ = SentryPose::Move;
            };
            std::tuple<GameState,NavState,RobotState> current(){
                return std::make_tuple(game_,nav_,robot_);
            }
        public:
            void change_GameState(GameState& game){
                this->game_ =game;
            }
            void change_RobotState(RobotState& robot){
                this->robot_ =robot;
            }
            void change_NavState(NavState& nav){
                this->nav_ =nav;
            }
            void change_SentryPose(SentryPose& sentry_pose){
                this->sentry_pose_=sentry_pose;
            }
        public:
            GameState game(){
                return this->game_;
            }
            NavState nav(){
                return this->nav_;
            }
            RobotState robot(){
                return this->robot_;
            }
            SentryPose sentry_pose(){
                return this->sentry_pose_;
            }
        private:
            GameState game_;
            NavState  nav_;
            RobotState robot_;
            SentryPose sentry_pose_;
    };

    namespace point {
        struct Point2D {
            double x ;
            double y ;
            Point2D(double x = 0, double y = 0) : x(x), y(y){}
        };
        Point2D HitOutPost = Point2D(1.0,2.0);
        Point2D Home = Point2D(1.0,2.0);
        Point2D Patroll_1 =Point2D(0,0);
        Point2D Patroll_2 =Point2D(0,0);
    }
}

#endif