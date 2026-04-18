#include "serial_node.h"

namespace io {
SerialNode::SerialNode(const std::string &serial_name, int &baud_rate,
                       int &max_try, const rclcpp::Node::SharedPtr node,
                       const std::string &socket_send_name,
                       const std::string &socket_receive_name,
                       const SendingMethod sending_method)
    : node_(node), running_(true), fsm_decision_(node),
      socket_send_name_(socket_send_name),
      socket_receive_name_(socket_receive_name),
      sending_method_(sending_method) {

  if (this->sending_method_ == SendingMethod::socket) {
    send_cmd_variant_ = SendSocketData();
    receive_data_variant_ = std::vector<ReceiveSocketData>();
    std::get<SendSocketData>(send_cmd_variant_).sof_0 = SOF0;
    std::get<SendSocketData>(send_cmd_variant_).sof_1 = SOF1;
    // std::get<SendSocketData>(send_cmd_variant_).sentry_pose=3;
    std::get<SendSocketData>(send_cmd_variant_).v_x = 0;
    std::get<SendSocketData>(send_cmd_variant_).v_y = 0;
    std::get<SendSocketData>(send_cmd_variant_).w_z = 0;
    std::get<std::vector<ReceiveSocketData>>(receive_data_variant_)
        .reserve(100);
  } else if (this->sending_method_ == SendingMethod::serial) {
    send_cmd_variant_ = SendData();
    receive_data_variant_ = std::vector<ReceiveData>();
    std::get<SendData>(send_cmd_variant_).sof_0 = SOF0;
    std::get<SendData>(send_cmd_variant_).sof_1 = SOF1;
    // std::get<SendData>(send_cmd_variant_).sentry_pose=3;
    std::get<SendData>(send_cmd_variant_).v_x = 0;
    std::get<SendData>(send_cmd_variant_).v_y = 0;
    std::get<SendData>(send_cmd_variant_).w_z = 0;
    std::get<SendData>(send_cmd_variant_).crc16 = 0;
    std::get<std::vector<ReceiveSocketData>>(receive_data_variant_)
        .reserve(100);
  }

  serial_driver =
      std::make_shared<io::SerialDriver>(serial_name, baud_rate, max_try);
  RCLCPP_INFO(node_->get_logger(),
              "node inner socket_send_name:%s socket_receive_name:%s ",
              socket_send_name.c_str(), socket_receive_name.c_str());
  this->is_open_serial =
      this->serial_driver->open_socket(socket_receive_name_, socket_send_name_);
  // this->is_open_serial =
  //    this->serial_driver->open_serial(serial_name, baud_rate);

  std::cout << "is_open_serial:" << this->is_open_serial << std::endl;
  if (!is_open_serial) {
    // serial_driver->reopen(serial_name, baud_rate, max_try);
  }
  this->cmd_sub_ = node_->create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", 10,
      std::bind(&SerialNode::cmd_callback, this, std::placeholders::_1));
  this->rm_data_pub_ =
      node_->create_publisher<rm_interfaces::msg::RmData>("rm_data", 10);
  recv_thread_ = std::thread(&SerialNode::read_callback, this);
}
void SerialNode::read_callback() {
  const int timeout_ms = 10; // 超时时间
  const int sleep_ms = 5;    // 无数据时的休眠时间

  while (running_) {
    if (this->sending_method_ == SendingMethod::socket) {
      this->read_socket_data();
    } else if (this->sending_method_ == SendingMethod::serial) {
      this->read_serial_data();
    }
  }
}
void SerialNode::cmd_callback(geometry_msgs::msg::Twist::SharedPtr cmd_data) {
  double now = std::chrono::duration<double>(
                   std::chrono::steady_clock::now().time_since_epoch())
                   .count();
  if (this->sending_method_ == SendingMethod::serial) {
    std::get<SendData>(send_cmd_variant_).sof_0 = SOF0;
    std::get<SendData>(send_cmd_variant_).sof_1 = SOF1;
    // std::get<SendData>(send_cmd_variant_).sentry_pose=3;
    std::get<SendData>(send_cmd_variant_).v_x = cmd_data->linear.x;
    std::get<SendData>(send_cmd_variant_).v_y = cmd_data->linear.y;
    std::get<SendData>(send_cmd_variant_).w_z = cmd_data->angular.z;
    RCLCPP_INFO(node_->get_logger(), "serial 发送 cmd vx: %f ,vy : %f,wz : %f",
                std::get<SendData>(send_cmd_variant_).v_x,
                std::get<SendData>(send_cmd_variant_).v_y,
                std::get<SendData>(send_cmd_variant_).w_z);
    // RCLCPP_INFO(node_->get_logger(), "serial 发送 crc16: 0x%04X",
    //             send_cmd.crc16);
    std::get<SendData>(send_cmd_variant_).crc16 =
        std::get<SendData>(send_cmd_variant_).calculateCRC16();
    serial_driver->send_serial(std::get<SendData>(send_cmd_variant_));
    plotter_debug_cmd(now, std::get<SendData>(send_cmd_variant_).v_x,
                      std::get<SendData>(send_cmd_variant_).v_y,
                      std::get<SendData>(send_cmd_variant_).w_z);

  } else if (this->sending_method_ == SendingMethod::socket) {
    std::get<SendSocketData>(send_cmd_variant_).sof_0 = SOF0;
    std::get<SendSocketData>(send_cmd_variant_).sof_1 = SOF1;
    // std::get<SendSocketData>(send_cmd_variant_).sentry_pose=3;
    std::get<SendSocketData>(send_cmd_variant_).v_x = cmd_data->linear.x;
    std::get<SendSocketData>(send_cmd_variant_).v_y = cmd_data->linear.y;
    std::get<SendSocketData>(send_cmd_variant_).w_z = cmd_data->angular.z;
    RCLCPP_INFO(node_->get_logger(), "serial 发送 cmd vx: %f ,vy : %f,wz : %f",
                std::get<SendSocketData>(send_cmd_variant_).v_x,
                std::get<SendSocketData>(send_cmd_variant_).v_y,
                std::get<SendSocketData>(send_cmd_variant_).w_z);
    // RCLCPP_INFO(node_->get_logger(), "serial 发送 crc16: 0x%04X",
    //             send_cmd.crc16);
    serial_driver->send_socket(std::get<SendSocketData>(send_cmd_variant_));
    plotter_debug_cmd(now, std::get<SendSocketData>(send_cmd_variant_).v_x,
                      std::get<SendSocketData>(send_cmd_variant_).v_y,
                      std::get<SendSocketData>(send_cmd_variant_).w_z);
  }
};
void SerialNode::read_socket_data() {
  const int timeout_ms = 10; // 超时时间
  const int sleep_ms = 5;    // 无数据时的休眠时间
  std::get<std::vector<ReceiveSocketData>>(receive_data_variant_).clear();

  // 接收所有可用数据包
  if (serial_driver->receive_all_socket(
          std::get<std::vector<ReceiveSocketData>>(receive_data_variant_),
          timeout_ms)) {
    if (!std::get<std::vector<ReceiveSocketData>>(receive_data_variant_)
             .empty()) {
      std::lock_guard<std::mutex> lock(data_mutex_);

      // 发布所有接收到的数据包
      for (const auto &packet :
           std::get<std::vector<ReceiveSocketData>>(receive_data_variant_)) {

        double now = std::chrono::duration<double>(
                         std::chrono::steady_clock::now().time_since_epoch())
                         .count();
        rm_data_.current_hp = packet.current_hp;
        rm_data_.game_progress = packet.game_progress;
        rm_data_.projectile_allowance_17mm = packet.projectile_allowance;
        receive_speed_.vx = packet.vx;
        receive_speed_.vy = packet.vy;
        receive_speed_.wz = packet.wz;
        rm_data_pub_->publish(rm_data_);
        // RCLCPP_INFO(node_->get_logger(), "Received - HP: %d, Progress:
        // %d,projectile_allowance: % d " , packet.current_hp,
        //                   packet.game_progress,
        //               packet.projectile_allowance);
        plotter_debug_receive(now);
        if (this->is_decision_) {
          // RCLCPP_INFO(node_->get_logger(), "Progress:
          // %d:",rm_data_.game_progress);
          fsm_decision_.decision(rm_data_.game_progress, rm_data_.current_hp,
                                 rm_data_.projectile_allowance_17mm);
        }
      }
    }
  } else {
    // 没有数据时短暂休眠，避免 CPU 空转
    std::this_thread::sleep_for(std::chrono::milliseconds(sleep_ms));
  }
}
void SerialNode::read_serial_data() {
  const int timeout_ms = 10; // 超时时间
  const int sleep_ms = 5;    // 无数据时的休眠时间
  std::get<std::vector<ReceiveData>>(receive_data_variant_).clear();

  // 接收所有可用数据包
  if (serial_driver->receive_all_serial(
          std::get<std::vector<ReceiveData>>(receive_data_variant_),
          timeout_ms)) {
    if (!std::get<std::vector<ReceiveData>>(receive_data_variant_).empty()) {
      std::lock_guard<std::mutex> lock(data_mutex_);

      // 发布所有接收到的数据包
      for (const auto &packet :
           std::get<std::vector<ReceiveData>>(receive_data_variant_)) {

        double now = std::chrono::duration<double>(
                         std::chrono::steady_clock::now().time_since_epoch())
                         .count();
        rm_data_.current_hp = packet.current_hp;
        rm_data_.game_progress = packet.game_progress;
        rm_data_.projectile_allowance_17mm = packet.projectile_allowance;
        receive_speed_.vx = packet.vx;
        receive_speed_.vy = packet.vy;
        receive_speed_.wz = packet.wz;
        rm_data_pub_->publish(rm_data_);
        // RCLCPP_INFO(node_->get_logger(), "Received - HP: %d, Progress:
        // %d,projectile_allowance: % d " , packet.current_hp,
        //                   packet.game_progress,
        //               packet.projectile_allowance);
        plotter_debug_receive(now);
        if (this->is_decision_) {
          // RCLCPP_INFO(node_->get_logger(), "Progress:
          // %d:",rm_data_.game_progress);
          fsm_decision_.decision(rm_data_.game_progress, rm_data_.current_hp,
                                 rm_data_.projectile_allowance_17mm);
        }
      }
    }
  } else {
    // 没有数据时短暂休眠，避免 CPU 空转
    std::this_thread::sleep_for(std::chrono::milliseconds(sleep_ms));
  }
}
void SerialNode::plotter_debug_cmd(double &now, double vx, double vy,
                                   double wz) {
  nlohmann::json cmd_data_json;
  cmd_data_json["ts"] = now;
  cmd_data_json["cmd_vx"] = vx;
  cmd_data_json["cmd_vy"] = vy;
  cmd_data_json["cmd_wz"] = wz;
  plotter.plot(cmd_data_json);
}
void SerialNode::plotter_debug_receive(double &now) {
  nlohmann::json true_speed_data;
  true_speed_data["ts"] = now;
  true_speed_data["true_vx"] = receive_speed_.vx;
  true_speed_data["true_vy"] = receive_speed_.vy;
  true_speed_data["true_wz"] = receive_speed_.wz;
  plotter.plot(true_speed_data);
}
} // namespace io