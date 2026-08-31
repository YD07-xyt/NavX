#pragma once
#include<rclcpp/rclcpp.hpp>
#include<geometry_msgs/msg/pose_stamped.hpp>
#include <action_msgs/msg/goal_status_array.hpp>
#include "patrol.h"
#include"type.h"
namespace decision {

    class FSMRos2 {
        public:
            FSMRos2 (rclcpp::Node::SharedPtr node);
            void decision(int is_game,int current_hp,int projectile_allowance);
            void decision(int is_game,int current_hp,int projectile_allowance,int is_enemy_outpost_destroyed,int game_time);
            void init_start_time(std::chrono::steady_clock::time_point StartTime){
                waitStartTime=StartTime;
            };
        private:
            void is_good_robot_condition(int current_hp,int projectile_allowance);
            void state_enemy_outpost(int is_enemy_outpost_destroyed,int game_time);
            void hit_enemy_outpost();
            void SwitchpatrolState();
            void ExecuteGameTask();
            void go_home();
            void patrolA();
            void patrolB();
        private:
            void advancePatrolIndex();
            Point selectTarget(int current_hp, int projectile_allowance);
        private:
            FSMConfig fsm_config_;
            rclcpp::Node::SharedPtr node_;
            void pub_goal(Point goal);
            rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
            rclcpp::Subscription<action_msgs::msg::GoalStatusArray>::SharedPtr nav2_status_sub_;
            void nav2_status_callback(const action_msgs::msg::GoalStatusArray& msg);
            void printf_nav2_state();
        public:
            Patrol patrol_;
            EnemyOutpostState enemy_outpost_state_=EnemyOutpostState::not_destroyed;
            GameTask current_game_task_=GameTask::Free;
            Nav2State nav2_state_=Nav2State::idle;
            RobotState robot_state_=RobotState::Normal;
            PatrolState patrol_state_=PatrolState::Free;
            StateIsGoHome state_is_go_home_;
            bool is_temp_hit_point=false;
            double wait_time=0;
            double nav_time=0;
            std::chrono::steady_clock::time_point nav_end_time;
            std::chrono::steady_clock::time_point nav_start_time;
            std::chrono::steady_clock::time_point wait_start_time;
            bool goal_sent_ = false;  // 是否已发送当前目标点
            Point current_goal_point_=Point(0,0,0);  // 当前正在导航的目标点
        public:
            GoalPoint goal_point_sum_;
            int nav2_status_=0;
            int current_patrol_index_ = 0; 
            decision::Point last_sent_goal_;
            std::chrono::steady_clock::time_point waitStartTime;
            std::chrono::steady_clock::time_point nav_start_time_;
            std::chrono::steady_clock::time_point nav_end_time_;
            float wait_point1_time_ =5.0;
            float wait_point2_time_ =5.0;
            
    };
}