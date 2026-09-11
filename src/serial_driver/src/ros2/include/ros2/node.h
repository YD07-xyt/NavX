#pragma once

#include <chrono>
#include <memory>
#include <optional>
#include <rclcpp/node.hpp>
#include <std_msgs/msg/detail/bool__struct.hpp>
#include <string>
#include <variant>
#include <vector>
//ros2
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/timer.hpp>
#include <std_msgs/msg/int16.hpp>
//
#include <nlohmann/detail/json_pointer.hpp>

//serial
#include "serial/serial.h"
#include "serial/config.hpp"
#include "tools/plotter.hpp"
//rm_decision
#include "rm_decision/config.hpp"
#include "rm_decision/rm_decision.hpp"
#include "rm_decision/api.hpp"
namespace ros2 {

class SerialNode {
public:
    SerialNode(
        const io::SerialConfig& serial_config,
        const rclcpp::Node::SharedPtr node,
        const bt::DecisionConfig& decision_config
    );
    ~SerialNode() {
        running_ = false;
        if (recv_thread_.joinable()) {
            recv_thread_.join();
        }
        RCLCPP_INFO(node_->get_logger(), "SerialNode destroyed");
    }

private:
    bt::RmDecision rm_decision_;
    bt::DecisionConfig decision_config_;
    bt::Game2Decision game_data;
    bt::Nav2Decision nav_data;
    std::optional<bt::Decision2Game> decision2game_data;
    void rm_bt_callback();
    void init_bt();
    
private:
    io::SerialConfig serial_config_;

private:
    std::shared_ptr<io::SerialDriver> serial_driver;
    std::variant<io::SendSerialData, io::SendSocketData> send_cmd_variant_;
    std::variant<std::vector<io::ReceiveSerialData>, std::vector<io::ReceiveSocketData>> receive_data_variant_;
    // rm_interfaces::msg::RmData rm_data_;
    struct Speed {
        double vx;
        double vy;
        double wz;
    } receive_speed_;

    std::thread recv_thread_;

    std::mutex data_mutex_;

private:
    tools::Plotter plotter;

public:
    bool is_decision_ = true;

public:
private:
    // serial
    void init_serial_driver();

private:
    rclcpp::Node::SharedPtr node_;
    std::atomic<bool> running_;
    std::chrono::steady_clock::time_point waitStartTime;

private:
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr nav_feedback_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr goal_pub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr fold_sub_;
    rclcpp::TimerBase::SharedPtr bt_timer_;
    rclcpp::TimerBase::SharedPtr send_timer_;
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg){};
    void nav_feedback_callback(const std_msgs::msg::Int16::SharedPtr msg);
    void cmd_callback(const geometry_msgs::msg::Twist::SharedPtr cmd_data);
    void fold_callback(const std_msgs::msg::Bool::SharedPtr msg);
    void send_callback();
    void read_callback();
    void read_socket_data();
    void read_serial_data();
    void plotter_debug_cmd(double& now, double vx, double vy, double wz);
    void plotter_debug_receive(double& now);
};
} // namespace io