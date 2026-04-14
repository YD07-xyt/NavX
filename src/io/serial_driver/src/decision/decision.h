#pragma once
#include<rclcpp/rclcpp.hpp>
#include<geometry_msgs/msg/pose_stamped.hpp>
#include <action_msgs/msg/goal_status_array.hpp>
namespace decision {
    struct Point{
        double x;
        double y;
        double yaw;
        Point(double x,double y,double yaw):x(x),y(y),yaw(yaw){};
    };
    struct GoalPoint{
        Point Patrol1=Point(1,1,1);
        Point Patrol2=Point(2,2,2);
        Point home=Point(0.229,0.807,0.0);
    };
    class FSM {
        public:
            FSM(rclcpp::Node::SharedPtr node);
            void decision(int is_game,int current_hp,int projectile_allowance);
            void advancePatrolIndex();
            Point selectTarget(int current_hp, int projectile_allowance);
        private:
            std::string map_tf_name_="map";
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

    };
}