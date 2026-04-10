#pragma once

#include <chrono>
#include <geometry_msgs/msg/twist.hpp>
#include <memory>
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>

#include <rclcpp/timer.hpp>
#include <rm_interfaces/msg/detail/rm_data__struct.hpp>
#include <rm_interfaces/msg/rm_data.hpp>

#include "src/packet_typedef.h"
#include "src/serial.h"
namespace io {
class SerialNode {
public:
  SerialNode(const std::string &serial_name, int &baud_rate, int &max_try,
             const rclcpp::Node::SharedPtr node)
      : node_(node), running_(true) {

    serial_driver =
        std::make_shared<io::SerialDriver>(serial_name, baud_rate, max_try);
    this->is_open_serial =
        this->serial_driver->open_socket(serial_name, baud_rate);

    receive_data.reserve(100);    
    
    if (!is_open_serial) {
      // serial_driver->reopen(serial_name,baud_rate,max_try);
    } else {
      this->cmd_sub_ = node_->create_subscription<geometry_msgs::msg::Twist>(
          "cmd_vel", 10,
          std::bind(&SerialNode::cmd_callback, this, std::placeholders::_1));
      this->rm_data_pub_ =
          node_->create_publisher<rm_interfaces::msg::RmData>("rm_data", 10);
      recv_thread_ = std::thread(&SerialNode::read_callback, this);
    }
  }
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
  std::shared_ptr<io::SerialDriver> serial_driver;
  SendData send_cmd;
  std::vector<ReceiveData> receive_data;
  rm_interfaces::msg::RmData rm_data_;
  std::thread recv_thread_;
  
  std::mutex data_mutex_;

private:
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  /*TODO: pub receive*/
  rclcpp::Publisher<rm_interfaces::msg::RmData>::SharedPtr rm_data_pub_;
  void read_callback() {
    const int timeout_ms = 10; // 超时时间
    const int sleep_ms = 5;    // 无数据时的休眠时间

    while (running_) {
      receive_data.clear();
      // 接收所有可用数据包
      if (serial_driver->receive_all(receive_data, timeout_ms)) {
        if (!receive_data.empty()) {
          std::lock_guard<std::mutex> lock(data_mutex_);

          // 发布所有接收到的数据包
          for (const auto &packet : receive_data) {
            rm_data_.current_hp = packet.current_hp;
            rm_data_.game_progress = packet.game_progress;
            rm_data_.projectile_allowance_17mm = packet.projectile_allowance;

            rm_data_pub_->publish(rm_data_);

            RCLCPP_INFO(node_->get_logger(),
                        "Received - HP: %d, Progress: %d, Ammo: %d",
                        packet.current_hp, packet.game_progress,
                        packet.projectile_allowance);
          }
        }
      } else {
        // 没有数据时短暂休眠，避免 CPU 空转
        std::this_thread::sleep_for(std::chrono::milliseconds(sleep_ms));
      }
    }
  }
  void cmd_callback(geometry_msgs::msg::Twist::SharedPtr cmd_data) {
    send_cmd.v_x = cmd_data->linear.x;
    send_cmd.v_y = cmd_data->linear.y;
    send_cmd.w_z = cmd_data->angular.z;
    RCLCPP_INFO(node_->get_logger(), "serial 发送 cmd vx: %f ,vy : %f,wz : %f",
                send_cmd.v_x, send_cmd.v_y, send_cmd.w_z);
    serial_driver->send(send_cmd);
  };
};
} // namespace io