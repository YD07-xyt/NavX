#pragma once
#include<rclcpp/rclcpp.hpp>
#include<geometry_msgs/msg/pose_stamped.hpp>
#include <action_msgs/msg/goal_status_array.hpp>
#include"type.h"
namespace decision {
    class FSMRos2 {
        public:
            FSMRos2 (rclcpp::Node::SharedPtr node);
            void decision(int is_game,int current_hp,int projectile_allowance);
            void init_start_time(std::chrono::steady_clock::time_point StartTime){
                waitStartTime=StartTime;
            };
        private:
            void advancePatrolIndex();
            Point selectTarget(int current_hp, int projectile_allowance);
        private:
            FSMConfig fsm_config_;
            rclcpp::Node::SharedPtr node_;
            void pub_goal(Point goal);
            rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
            rclcpp::Subscription<action_msgs::msg::GoalStatusArray>::SharedPtr nav2_status_sub_;
            void nav2_status_callback(const action_msgs::msg::GoalStatusArray msg);
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