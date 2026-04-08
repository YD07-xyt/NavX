#pragma once
#include<rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/timer.hpp>
#include <rm_interfaces/msg/detail/aim2_nav__struct.hpp>
#include <rm_interfaces/msg/detail/nav2_aim__struct.hpp>
#include <vector>
#include"rm_interfaces/msg/aim2_nav.hpp"
#include"rm_interfaces/msg/nav2_aim.hpp"
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
    class FSM : public rclcpp::Node{
        public:
            explicit FSM();

        private:
            rclcpp::Publisher<rm_interfaces::msg::Nav2Aim>::SharedPtr nav2aim_pub_;
            rclcpp::Subscription<rm_interfaces::msg::Aim2Nav>::SharedPtr aim2nav_sub_;  
            void aim2nav_sub_callback(const rm_interfaces::msg::Aim2Nav& msg);
            void nav2aim_pub_callback();
            rclcpp::TimerBase::SharedPtr pub_timer_;
            std::vector<Aim2NavData> aim2nav_data_;
    };
}