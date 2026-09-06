#pragma once

#include <chrono>
#include <geometry_msgs/msg/twist.hpp>
#include <memory>
#include <nlohmann/detail/json_pointer.hpp>
#include <optional>
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>

#include <rclcpp/timer.hpp>

#include <string>
#include <variant>
#include <vector>

//#include "serial/include/serial/packet_typedef.h"
#include "serial/include/serial/serial.h"
#include "tools/include/plotter.hpp"

#include "rm_decision/include/config.hpp"
#include "rm_decision/include/rm_decision.hpp"
//#include <rm_interfaces/msg/rm_data.hpp>
namespace io {
enum SendingMethod {
  serial,
  socket,
};
class SerialNode {
public:
  SerialNode(const std::string &serial_name, int &baud_rate, int &max_try,
             const rclcpp::Node::SharedPtr node,
             const std::string &socket_send_name,
             const std::string &socket_receive_name,
             const SendingMethod sending_method,
             const bt::DecisionConfig &decision_config);
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
  bt::Topics2Blackboard::GameData game_data;
  rclcpp::TimerBase::SharedPtr bt_timer_;
  void rm_bt_callback();
  void init_bt();
  struct {
    std::optional<std::string> nav_model;
  } bt_value_;

private:
  rclcpp::Node::SharedPtr node_;
  std::atomic<bool> running_;
  bool is_open_serial;
  std::chrono::steady_clock::time_point waitStartTime;

private:
  std::shared_ptr<io::SerialDriver> serial_driver;
  bool serial_socket = false;
  SendingMethod sending_method_ = SendingMethod::socket;
  std::variant<SendData, SendSocketData> send_cmd_variant_;
  std::variant<std::vector<ReceiveData>, std::vector<ReceiveSocketData>>
      receive_data_variant_;
  // rm_interfaces::msg::RmData rm_data_;
  struct speed {
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
  std::string socket_send_name_;
  std::string socket_receive_name_;

public:


private:
  // serial
  void set_serial();

private:
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

  rclcpp::TimerBase::SharedPtr send_timer_;
  void OdomCallback(const nav_msgs::msg::Odometry &msg);
  void send_callback();
  void read_callback();
  void cmd_callback(geometry_msgs::msg::Twist::SharedPtr cmd_data);
  void read_socket_data();
  void read_serial_data();
  void plotter_debug_cmd(double &now, double vx, double vy, double wz);
  void plotter_debug_receive(double &now);
};
} // namespace io