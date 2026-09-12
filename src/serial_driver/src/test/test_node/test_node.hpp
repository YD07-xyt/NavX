#pragma once
#include "rm_decision/api.hpp"
#include "rm_decision/config.hpp"
#include "rm_decision/rm_decision.hpp"
#include <action_msgs/msg/goal_status.hpp>
#include <action_msgs/msg/goal_status_array.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/timer.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/detail/int16__struct.hpp>
#include <std_msgs/msg/int16.hpp>
namespace test {
class TestNode {
public:
    explicit TestNode(rclcpp::Node::SharedPtr node, const bt::DecisionConfig& decision_config):
        node_(node),
        start_time_(std::chrono::steady_clock::now()),
        rm_decision_(decision_config)
        {
        logger = logger::create_colored_logger("test");
        this->cmd_sub_ = node_->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel",
            10,
            [this](const geometry_msgs::msg::Twist::SharedPtr msg) { cmd_callback(msg); }
        );
        this->nav_feedback_sub_ = node_->create_subscription<std_msgs::msg::Int16>(
            "/ma_nav/nav_feedback",
            10,
            [this](const std_msgs::msg::Int16::SharedPtr msg) { nav_feedback_callback(msg); }
        );
        this->fold_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
            "/ma_nav/fold",
            10,
            [this](const std_msgs::msg::Bool::SharedPtr msg) { this->fold_callback(msg); }
        );
        this->goal_pub_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>("/goal_pose", 10);
        this->bt_timer_ = node_->create_wall_timer(std::chrono::milliseconds(10), [this]() { this->rm_bt_callback(); });
        this->game_timer_ =
            node_->create_wall_timer(std::chrono::milliseconds(5), [this]() { this->rm_game_callback(); });
        init_game();
    }

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
    rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr nav_feedback_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr fold_sub_;
    rclcpp::TimerBase::SharedPtr bt_timer_;
    rclcpp::TimerBase::SharedPtr game_timer_;
    void nav_feedback_callback(const std_msgs::msg::Int16::SharedPtr msg);
    void cmd_callback(const geometry_msgs::msg::Twist::SharedPtr cmd_data) {};
    void fold_callback(const std_msgs::msg::Bool::SharedPtr msg);
    void rm_bt_callback();
    void rm_game_callback();
    void init_game();

private:
    bt::Nav2Decision nav_data;
    bt::Game2Decision game_data;
    bt::RmDecision rm_decision_;
    std::chrono::steady_clock::time_point start_time_;
private:
    std::shared_ptr<spdlog::logger> logger;
};
} // namespace test