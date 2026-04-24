#pragma once

#include <chrono>
#include <geometry_msgs/msg/twist.hpp>
#include <memory>
#include <nlohmann/detail/json_pointer.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>

#include <rclcpp/timer.hpp>

#include <string>
#include <variant>
#include <vector>

#include "../decision/decision.h"
#include "../serial/packet_typedef.h"
#include "../serial/serial.h"
#include "../tools/plotter.hpp"
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
             const SendingMethod sending_method);
  ~SerialNode() {
    running_ = false;
    if (recv_thread_.joinable()) {
      recv_thread_.join();
    }
    RCLCPP_INFO(node_->get_logger(), "SerialNode destroyed");
  }

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
  //rm_interfaces::msg::RmData rm_data_;
  struct speed {
    double vx;
    double vy;
    double wz;
  } receive_speed_;

  std::thread recv_thread_;

  std::mutex data_mutex_;

private:
  tools::Plotter plotter;
  decision::FSMRos2 fsm_decision_;

public:
  bool is_decision_ = true;
  std::string socket_send_name_;
  std::string socket_receive_name_;

public:
  void init_goal(decision::GoalPoint goal_point_sum) {
    fsm_decision_.goal_point_sum_ = goal_point_sum;
    fsm_decision_.patrol_.goal_point_sum_=goal_point_sum  ;
  };

private:
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  /*TODO: pub receive*/
  ////rclcpp::Publisher<rm_interfaces::msg::RmData>::SharedPtr rm_data_pub_;
  void read_callback();
  void cmd_callback(geometry_msgs::msg::Twist::SharedPtr cmd_data);
  void read_socket_data();
  void read_serial_data();
  void plotter_debug_cmd(double &now, double vx, double vy, double wz);
  void plotter_debug_receive(double &now);
};
} // namespace io