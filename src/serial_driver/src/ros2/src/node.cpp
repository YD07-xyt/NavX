#include "ros2/node.h"
#include "rm_decision/include/rm_decision.hpp"
#include <chrono>
#include <spdlog/spdlog.h>
#include <string>

namespace io {
SerialNode::SerialNode(const std::string &serial_name, int &baud_rate,
                       int &max_try, const rclcpp::Node::SharedPtr node,
                       const std::string &socket_send_name,
                       const std::string &socket_receive_name,
                       const SendingMethod sending_method,
                       const bt::DecisionConfig &decision_config)
    : node_(node), running_(true),
      socket_send_name_(socket_send_name),
      socket_receive_name_(socket_receive_name),
      sending_method_(sending_method),
      waitStartTime(std::chrono::steady_clock::now()),
      rm_decision_(node, decision_config) {

  set_serial();
  init_bt();
  serial_driver =
      std::make_shared<io::SerialDriver>(serial_name, baud_rate, max_try);

  spdlog::info("node inner socket_send_name:{} socket_receive_name:{} ",
               socket_send_name.c_str(), socket_receive_name.c_str());

  this->is_open_serial =
      this->serial_driver->open_socket(socket_receive_name_, socket_send_name_);
  // this->is_open_serial =
  //    this->serial_driver->open_serial(serial_name, baud_rate);

  spdlog::info("is_open_serial:{}", this->is_open_serial);
  if (!is_open_serial) {
    // serial_driver->reopen(serial_name,baud_rate, max_try);
  }

  this->cmd_sub_ = node_->create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", 10,
      [this](auto && PH1) { cmd_callback(std::forward<decltype(PH1)>(PH1)); });
  this->bt_timer_ = node_->create_wall_timer(
      std::chrono::milliseconds(5), [this]() { this->rm_bt_callback(); });
  this->send_timer_ = node_->create_wall_timer(
      std::chrono::milliseconds(1), [this]() { this->send_callback(); });
  this->odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
      "/odometry", 10,
      [this](const nav_msgs::msg::Odometry &msg) { this->OdomCallback(msg); });
  recv_thread_ = std::thread(&SerialNode::read_callback, this);
}
void SerialNode::init_bt(){
  rm_decision_.setBlackboardValue("nav_model", "spinning");
  rm_decision_.setBlackboardValue("uphill_done", false);
  rm_decision_.setBlackboardValue("downhill_done", true);
}
void SerialNode::rm_bt_callback() {
  rm_decision_.tree_tick(game_data, std::chrono::milliseconds(1));
  bt_value_.nav_model =
      rm_decision_.getBlackboardValue<std::string>("nav_model");
};
void SerialNode::OdomCallback(const nav_msgs::msg::Odometry &msg) {
  Eigen::Quaterniond q;
  tf2::fromMsg(msg.pose.pose.orientation, q); // 需要 tf2_eigen

  // 转为旋转矩阵再取欧拉角 (ZYX 顺序得到 yaw, pitch, roll)
  Eigen::Vector3d euler = q.toRotationMatrix().eulerAngles(2, 1, 0);
  std::get<SendSocketData>(send_cmd_variant_).robot_yaw = euler(0);
  spdlog::debug("[serial_node]robot_yaw:{}",euler(0));
};
void SerialNode::send_callback() {
  if (bt_value_.nav_model.has_value()) {
    if (bt_value_.nav_model == "follow") {
      std::get<SendSocketData>(send_cmd_variant_).is_chassis_follow = 1;
      spdlog::info("[serial_node]nav_model is follow");
    } else if(bt_value_.nav_model == "downhill") {
      std::get<SendSocketData>(send_cmd_variant_).is_chassis_follow = 2;
      spdlog::info("[serial_node]nav_model is downhill");
    } else{
      std::get<SendSocketData>(send_cmd_variant_).is_chassis_follow = 0;
    }
  }
  // spdlog::debug("serial 发送 cmd vx: {} ,vy :{},wz: {}
  // ",std::get<SendSocketData>(send_cmd_variant_).v_x,
  //       std::get<SendSocketData>(send_cmd_variant_).v_y,
  //       std::get<SendSocketData>(send_cmd_variant_).w_z);
  // spdlog::debug("serial 发送 crc16:{}", send_cmd.crc16);
  serial_driver->send_socket(std::get<SendSocketData>(send_cmd_variant_));
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
    //   spdlog::info("serial 发送 cmd vx: {} ,vy : {},wz
    //   :{}",std::get<SendData>(send_cmd_variant_).v_x,
    //               std::get<SendData>(send_cmd_variant_).v_y,
    //               std::get<SendData>(send_cmd_variant_).w_z);
    //  spdlog::info("serial 发送 crc16: {}",
    //               send_cmd.crc16);
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
    float multiple = 1.2;
    std::get<SendSocketData>(send_cmd_variant_).v_x =
        multiple * cmd_data->linear.x;
    std::get<SendSocketData>(send_cmd_variant_).v_y =
        multiple * cmd_data->linear.y;
    std::get<SendSocketData>(send_cmd_variant_).w_z =
        multiple * cmd_data->angular.z;
    // std::get<SendSocketData>(send_cmd_variant_).is_chassis_follow = 1;
    // std::get<SendSocketData>(send_cmd_variant_).robot_yaw = 360.0;
    //  spdlog::info( "serial 发送 cmd vx: {} ,vy :{},wz:{}",multiple * cmd_data->linear.x,
    //               multiple * cmd_data->linear.y,
    //               multiple * cmd_data->angular.z);
    // //   spdlog::info("serial 发送 crc16:{}",send_cmd.crc16);
    // serial_driver->send_socket(std::get<SendSocketData>(send_cmd_variant_));
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
        receive_speed_.vx = packet.vx;
        receive_speed_.vy = packet.vy;
        receive_speed_.wz = packet.wz;
        // rm_data_pub_->publish(rm_data_);
        spdlog::debug("Received-HP:{},Progress:{},projectile_allowance:{},game_"
                      "time:{},is_enemy_outpost_destroyed:{}",
                      packet.current_hp, packet.game_progress,
                      packet.projectile_allowance, packet.game_time,
                      packet.is_enemy_outpost_destroyed);
        plotter_debug_receive(now);

        int temp = packet.is_enemy_outpost_destroyed;
        auto time = std::chrono::duration_cast<std::chrono::milliseconds>(
                        std::chrono::steady_clock::now() - waitStartTime)
                        .count() /
                    1000.0;
        // spdlog::debug("now time:{}",time);
        //   if(time>=400){
        //     temp=0;
        //    // RCLCPP_INFO(node_->get_logger(),"敌方哨站is已被击毁");
        //   }
        if (packet.game_time <= 330 && packet.game_time >= 1) {
          temp = 0;
        }
        if (this->is_decision_) {
          // spdlog::debufg( "Progress:
          // {}:",rm_data_.game_progress);

          // fsm_decision_.decision(packet.game_progress, packet.current_hp,
          //                        packet.projectile_allowance, temp,
          //                        packet.game_time);
          /*TODO:set Topics2Blackboard::GameData*/
          game_data.game_time = packet.game_time;
          game_data.is_game_start = packet.game_progress;

          if (packet.is_enemy_outpost_destroyed == 1) {
            game_data.current_enemy_outpost_hp = 300;
            game_data.ours_fort_occ_state= false;
          } else if(packet.is_enemy_outpost_destroyed == 0)  {
            game_data.current_enemy_outpost_hp = 0;
            game_data.ours_fort_occ_state= false;
          } else if(packet.is_enemy_outpost_destroyed == 3){
            game_data.current_enemy_outpost_hp = 300;
            game_data.ours_fort_occ_state= true;
            spdlog::info("ours fort is occupied");
          }
          game_data.current_hp = packet.current_hp;
          game_data.projectile_allowance = packet.projectile_allowance;

          // rm_decision_.tree_tick(game_data, std::chrono::milliseconds(10));
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
        // rm_data_.current_hp = packet.current_hp;
        // rm_data_.game_progress = packet.game_progress;
        receive_speed_.vx = packet.vx;
        receive_speed_.vy = packet.vy;
        receive_speed_.wz = packet.wz;
        // rm_data_pub_->publish(rm_data_);
        RCLCPP_INFO(node_->get_logger(),
                    "Received-HP:%d,Progress:%d,projectile_allowance:%d,game_"
                    "time:%d,is_enemy_outpost_destroyed :%d",
                    packet.current_hp, packet.game_progress,
                    packet.projectile_allowance, packet.game_time,
                    packet.is_enemy_outpost_destroyed);
        plotter_debug_receive(now);
        if (this->is_decision_) {
          // RCLCPP_INFO(node_->get_logger(), "Progress:
          // %d:",rm_data_.game_progress);
          // fsm_decision_.decision(packet.game_progress, packet.current_hp,
          //                        packet.projectile_allowance,
          //                        packet.is_enemy_outpost_destroyed,
          //                        packet.game_time);
          /*TODO:set Topics2Blackboard::GameData*/
          bt::Topics2Blackboard::GameData game_data;
          game_data.game_time = packet.game_time;
          game_data.is_game_start = packet.game_progress;
          if (packet.is_enemy_outpost_destroyed == true) {
            game_data.current_enemy_outpost_hp = 0;
          } else {
            game_data.current_enemy_outpost_hp = 300;
          }
          game_data.current_hp = packet.current_hp;
          game_data.projectile_allowance = packet.projectile_allowance;
          // rm_decision_.tree_tick(game_data, std::chrono::milliseconds(100));
        }
      }
    }
  } else {
    // 没有数据时短暂休眠，避免 CPU 空转
    std::this_thread::sleep_for(std::chrono::milliseconds(sleep_ms));
  }
}

void SerialNode::set_serial() {
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
};

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