#pragma once
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <geometry_msgs/msg/twist_stamped.hpp>
#include<rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/timer.hpp>
#include <string>
#include <vector>
#include"rm_interfaces/msg/aim2_nav.hpp"
#include"rm_interfaces/msg/nav2_aim.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "nav2_msgs/action/navigate_to_pose.hpp"

namespace decision {
    struct Nav2AimData{
        float vx;
        float vy;
        float wz;
    };
    struct Aim2NavData{
        uint16_t current_hp;                      
        uint8_t  game_progress;
        uint16_t projectile_allowance_17mm;      
    };
    struct Point{
        double x;
        double y;
        double yaw;
        Point(double x,double y,double yaw):x(x),y(y),yaw(yaw){};
    };
    struct GoalPoint{
        Point Patrol1=Point(1,1,1);
        Point Patrol2=Point(2,2,2);
        Point home=Point(3,3,3);
    };
    struct tf{
        std::string map="map";
        std::string world="world";
        std::string odom="odom";
    };
    enum NavState{
        Running,
        Succeeded,
    };
    class FSM : public rclcpp::Node{
        public:
            using NavigateToPose = nav2_msgs::action::NavigateToPose;
            using GoalHandle = rclcpp_action::ClientGoalHandle<NavigateToPose>;
            explicit FSM();

        private:
            Aim2NavData game_data_;
            GoalPoint goal_point_sum;
            std::string map_tf_name_;
            bool is_succeeded=false;
            bool is_goal_accepted=false;
        private:
            rclcpp_action::Client<NavigateToPose>::SharedPtr goal_client_;
            rclcpp::Publisher<rm_interfaces::msg::Nav2Aim>::SharedPtr nav2aim_pub_;
            rclcpp::Subscription<rm_interfaces::msg::Aim2Nav>::SharedPtr aim2nav_sub_;  
            rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
            rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
            void aim2nav_sub_callback(const rm_interfaces::msg::Aim2Nav& msg);
            void nav2aim_pub_callback();
            void cmd_sub_callback(const geometry_msgs::msg::Twist& msg);
            rclcpp::TimerBase::SharedPtr pub_timer_;
            std::vector<Aim2NavData> aim2nav_data_;
            void sendGoal_pub_callback();
            void sendGoal_action_callback();
            void resend_action_goal(Point point,std::string map_tf_name);
            void send_action_goal(Point point,std::string map_tf_name);
            void send_pub_Goal(Point goal_point,std::string map_tf_name);

            void goal_response_callback(const GoalHandle::SharedPtr& goal_handle);
            void feedback_callback(
                GoalHandle::SharedPtr,
                const std::shared_ptr<const NavigateToPose::Feedback> feedback);
            void result_callback(const GoalHandle::WrappedResult& result);
    };
}