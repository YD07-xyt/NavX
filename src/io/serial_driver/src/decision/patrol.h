#pragma once
#include "type.h"
#include <chrono>
#include <functional>
#include <rclcpp/node.hpp>
namespace decision {
    enum class PatrolState {
        Wait,
        Running,//正在巡逻
        Succeeded,//导航成功
        Free,
    };
    struct PatrolWaitTime{
        float wait_point1_time =5.0;
        float wait_point2_time =5.0;
        float wait_point3_time =5.0;
        float wait_point4_time =5.0;
    };
    class Patrol {
        public:
            Patrol(const rclcpp::Node::SharedPtr node):node_(node),goal(0,0,0){};
            void advancePatrolIndex();
            Point selectTarget();
            GoalPoint goal_point_sum_;
        private:
            PatrolState current_state_ = PatrolState::Free;
            Point goal;
            std::chrono::steady_clock::time_point wait_start_time_;
            PatrolWaitTime patrol_wait_time_;

            rclcpp::Node::SharedPtr node_;
            
            int current_patrol_index_ = 0; 
    };
}