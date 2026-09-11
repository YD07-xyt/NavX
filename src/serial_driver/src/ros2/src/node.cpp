#include "ros2/node.h"
#include "rm_decision/api.hpp"
#include "rm_decision/rm_decision.hpp"
#include "serial/packet_typedef.h"
#include <chrono>
#include <geometry_msgs/msg/detail/twist__struct.hpp>
#include <spdlog/spdlog.h>
#include <std_msgs/msg/detail/bool__struct.hpp>
#include <std_msgs/msg/detail/float64__struct.hpp>
#include <string>
#include <tf2_geometry_msgs/tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace ros2 {
SerialNode::SerialNode(
    const io::SerialConfig& serial_config,
    const rclcpp::Node::SharedPtr node,
    const bt::DecisionConfig& decision_config
):
    node_(node),
    running_(true),
    serial_config_(serial_config),
    waitStartTime(std::chrono::steady_clock::now()),
    rm_decision_(decision_config) {
    serial_driver = std::make_shared<io::SerialDriver>();
    init_serial_driver();
    init_bt();

    spdlog::info(
        "node inner socket_send_name:{} socket_receive_name:{} ",
        serial_config_.socket_send_name.c_str(),
        serial_config_.socket_receive_name.c_str()
    );

    auto is_open_serial =
        this->serial_driver->open_socket(serial_config_.socket_receive_name, serial_config_.socket_send_name);

    spdlog::info("is_open_serial:{}", is_open_serial);
    if (!is_open_serial) {
        // serial_driver->reopen(serial_name,baud_rate, max_try);
    }

    this->cmd_sub_ = node_->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel",
        10,
        [this](const geometry_msgs::msg::Twist::SharedPtr msg) { cmd_callback(msg); }
    );
    this->nav_feedback_sub_ = node_->create_subscription<std_msgs::msg::Int16>(
        "/ma_nav/nav_feedback",
        10,
        [this](const std_msgs::msg::Int16::SharedPtr msg) { this->nav_feedback_callback(msg); }
    );
    this->bt_timer_ = node_->create_wall_timer(std::chrono::milliseconds(5), [this]() { this->rm_bt_callback(); });
    this->send_timer_ = node_->create_wall_timer(std::chrono::milliseconds(1), [this]() { this->send_callback(); });
    this->odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
        "/odometry",
        10,
        [this](const nav_msgs::msg::Odometry::SharedPtr msg) { this->odom_callback(msg); }
    );
    this->fold_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
        "/ma_nav/fold",
        10,
        [this](const std_msgs::msg::Bool::SharedPtr msg) { this->fold_callback(msg); }
    );
    this->goal_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>("ma/goal", 10);
    recv_thread_ = std::thread(&SerialNode::read_callback, this);
}
void SerialNode::init_bt() {
    nav_data.nav_state = bt::NavState::IDLE;
}
void SerialNode::rm_bt_callback() {
    rm_decision_.tree_tick(game_data, nav_data, std::chrono::milliseconds(10));
    auto [toGame, toNav] = rm_decision_.get_data();
    if (toNav.has_value()) {
        geometry_msgs::msg::Twist goal;
        goal.linear.x = toNav->goal_point.x;
        goal.linear.y = toNav->goal_point.y;
        goal.angular.z = toNav->goal_point.yaw;
    }
    if (toGame.has_value()) {
        decision2game_data->sentry_model = toGame->sentry_model;
    }
};
void SerialNode::nav_feedback_callback(const std_msgs::msg::Int16::SharedPtr msg) {
    if (msg->data == 0) {
        nav_data.nav_state = bt::NavState::IDLE;
    } else if (msg->data == 1) {
        nav_data.nav_state = bt::NavState::RUNNING;
    } else if (msg->data == 2) {
        nav_data.nav_state = bt::NavState::FAILURE;
    } else if (msg->data == 3) {
        nav_data.nav_state = bt::NavState::SUCCEEDED;
    }
};
void SerialNode::fold_callback(const std_msgs::msg::Bool::SharedPtr msg) {
    if (serial_config_.sending_method == io::SendingMethod::SERIAL) {
        std::get<io::SendSerialData>(send_cmd_variant_).is_fold = msg->data;
    } else if (serial_config_.sending_method == io::SendingMethod::SOCKET) {
        std::get<io::SendSocketData>(send_cmd_variant_).is_fold = msg->data;
    }
}

void SerialNode::send_callback() {
    serial_driver->send_socket(std::get<io::SendSocketData>(send_cmd_variant_));
}
void SerialNode::read_callback() {
    while (running_) {
        if (serial_config_.sending_method == io::SendingMethod::SOCKET) {
            this->read_socket_data();
        } else if (serial_config_.sending_method == io::SendingMethod::SERIAL) {
            this->read_serial_data();
        }
    }
}
void SerialNode::cmd_callback(const geometry_msgs::msg::Twist::SharedPtr cmd_data) {
    double now = std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
    if (serial_config_.sending_method == io::SendingMethod::SERIAL) {
        std::get<io::SendSerialData>(send_cmd_variant_).sof_0 = SOF0;
        std::get<io::SendSerialData>(send_cmd_variant_).sof_1 = SOF1;
        // std::get<SendData>(send_cmd_variant_).sentry_pose=3;
        std::get<io::SendSerialData>(send_cmd_variant_).vx = cmd_data->linear.x;
        std::get<io::SendSerialData>(send_cmd_variant_).vy = cmd_data->linear.y;
        std::get<io::SendSerialData>(send_cmd_variant_).wz = cmd_data->angular.z;
        std::get<io::SendSerialData>(send_cmd_variant_).crc16 =
            std::get<io::SendSerialData>(send_cmd_variant_).calculate_crc16();
        serial_driver->send_serial(std::get<io::SendSerialData>(send_cmd_variant_));
        plotter_debug_cmd(
            now,
            std::get<io::SendSerialData>(send_cmd_variant_).vx,
            std::get<io::SendSerialData>(send_cmd_variant_).vy,
            std::get<io::SendSerialData>(send_cmd_variant_).wz
        );

    } else if (serial_config_.sending_method == io::SendingMethod::SOCKET) {
        std::get<io::SendSocketData>(send_cmd_variant_).sof_0 = SOF0;
        std::get<io::SendSocketData>(send_cmd_variant_).sof_1 = SOF1;
        // std::get<SendSocketData>(send_cmd_variant_).sentry_pose=3;
        float multiple = 1.2;
        std::get<io::SendSocketData>(send_cmd_variant_).vx = multiple * cmd_data->linear.x;
        std::get<io::SendSocketData>(send_cmd_variant_).vy = multiple * cmd_data->linear.y;
        std::get<io::SendSocketData>(send_cmd_variant_).wz = multiple * cmd_data->angular.z;
        // std::get<SendSocketData>(send_cmd_variant_).is_chassis_follow = 1;
        // std::get<SendSocketData>(send_cmd_variant_).robot_yaw = 360.0;
        //  spdlog::info( "serial 发送 cmd vx: {} ,vy :{},wz:{}",multiple * cmd_data->linear.x,
        //               multiple * cmd_data->linear.y,
        //               multiple * cmd_data->angular.z);
        // //   spdlog::info("serial 发送 crc16:{}",send_cmd.crc16);
        // serial_driver->send_socket(std::get<SendSocketData>(send_cmd_variant_));
        plotter_debug_cmd(
            now,
            std::get<io::SendSocketData>(send_cmd_variant_).vx,
            std::get<io::SendSocketData>(send_cmd_variant_).vy,
            std::get<io::SendSocketData>(send_cmd_variant_).wz
        );
    }
};
void SerialNode::read_socket_data() {
    const int timeout_ms = 10; // 超时时间
    const int sleep_ms = 5; // 无数据时的休眠时间
    std::get<std::vector<io::ReceiveSocketData>>(receive_data_variant_).clear();

    // 接收所有可用数据包
    if (serial_driver
            ->receive_all_socket(std::get<std::vector<io::ReceiveSocketData>>(receive_data_variant_), timeout_ms)) {
        if (!std::get<std::vector<io::ReceiveSocketData>>(receive_data_variant_).empty()) {
            std::lock_guard<std::mutex> lock(data_mutex_);

            // 发布所有接收到的数据包
            for (const auto& packet: std::get<std::vector<io::ReceiveSocketData>>(receive_data_variant_)) {
                double now = std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
                receive_speed_.vx = packet.vx;
                receive_speed_.vy = packet.vy;
                receive_speed_.wz = packet.wz;
                // rm_data_pub_->publish(rm_data_);
                spdlog::debug(
                    "Received-HP:{},Progress:{},projectile_allowance:{},game_"
                    "time:{},is_enemy_outpost_destroyed:{}",
                    packet.current_hp,
                    packet.game_progress,
                    packet.projectile_allowance,
                    packet.game_time,
                    packet.is_enemy_outpost_destroyed
                );
                plotter_debug_receive(now);

                int temp = packet.is_enemy_outpost_destroyed;
                auto time = std::chrono::duration_cast<std::chrono::milliseconds>(
                                std::chrono::steady_clock::now() - waitStartTime
                            )
                                .count()
                    / 1000.0;
                if (packet.game_time <= 330 && packet.game_time >= 1) {
                    temp = 0;
                }
                if (this->is_decision_) {
                    /*TODO:set Topics2Blackboard::GameData*/

                    game_data.game_time = packet.game_time;
                    game_data.is_game_start = packet.game_progress;

                    if (packet.is_enemy_outpost_destroyed == 1) {
                        game_data.current_enemy_outpost_hp = 300;
                        game_data.ours_fort_occ_state = false;
                    } else if (packet.is_enemy_outpost_destroyed == 0) {
                        game_data.current_enemy_outpost_hp = 0;
                        game_data.ours_fort_occ_state = false;
                    } else if (packet.is_enemy_outpost_destroyed == 3) {
                        game_data.current_enemy_outpost_hp = 300;
                        game_data.ours_fort_occ_state = true;
                        spdlog::info("ours fort is occupied");
                    }
                    game_data.current_hp = packet.current_hp;
                    game_data.projectile_allowance = packet.projectile_allowance;
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
    const int sleep_ms = 5; // 无数据时的休眠时间
    std::get<std::vector<io::ReceiveSerialData>>(receive_data_variant_).clear();

    // 接收所有可用数据包
    if (serial_driver
            ->receive_all_serial(std::get<std::vector<io::ReceiveSerialData>>(receive_data_variant_), timeout_ms)) {
        if (!std::get<std::vector<io::ReceiveSerialData>>(receive_data_variant_).empty()) {
            std::lock_guard<std::mutex> lock(data_mutex_);

            // 发布所有接收到的数据包
            for (const auto& packet: std::get<std::vector<io::ReceiveSerialData>>(receive_data_variant_)) {
                double now = std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
                // rm_data_.current_hp = packet.current_hp;
                // rm_data_.game_progress = packet.game_progress;
                receive_speed_.vx = packet.vx;
                receive_speed_.vy = packet.vy;
                receive_speed_.wz = packet.wz;
                // rm_data_pub_->publish(rm_data_);
                RCLCPP_INFO(
                    node_->get_logger(),
                    "Received-HP:%d,Progress:%d,projectile_allowance:%d,game_"
                    "time:%d,is_enemy_outpost_destroyed :%d",
                    packet.current_hp,
                    packet.game_progress,
                    packet.projectile_allowance,
                    packet.game_time,
                    packet.is_enemy_outpost_destroyed
                );
                plotter_debug_receive(now);
                if (this->is_decision_) {
                    /*TODO:set Topics2Blackboard::GameData*/

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

void SerialNode::init_serial_driver() {
    if (serial_config_.sending_method == io::SendingMethod::SOCKET) {
        serial_driver
            ->init_socket(serial_config_.socket_receive_name, serial_config_.socket_send_name, serial_config_.max_try);
        send_cmd_variant_ = io::SendSocketData();
        receive_data_variant_ = std::vector<io::ReceiveSocketData>();
        std::get<io::SendSocketData>(send_cmd_variant_).sof_0 = SOF0;
        std::get<io::SendSocketData>(send_cmd_variant_).sof_1 = SOF1;
        // std::get<SendSocketData>(send_cmd_variant_).sentry_pose=3;
        std::get<io::SendSocketData>(send_cmd_variant_).vx = 0;
        std::get<io::SendSocketData>(send_cmd_variant_).vy = 0;
        std::get<io::SendSocketData>(send_cmd_variant_).wz = 0;
        std::get<std::vector<io::ReceiveSocketData>>(receive_data_variant_).reserve(1000);
    } else if (serial_config_.sending_method == io::SendingMethod::SERIAL) {
        //set param
        serial_driver->init_serial(serial_config_.serial_name, serial_config_.baud_rate, serial_config_.max_try);
        //set receive

        receive_data_variant_ = std::vector<io::ReceiveSerialData>();
        //set send
        send_cmd_variant_ = io::SendSerialData();
        std::get<io::SendSerialData>(send_cmd_variant_).sof_0 = SOF0;
        std::get<io::SendSerialData>(send_cmd_variant_).sof_1 = SOF1;
        // std::get<SendData>(send_cmd_variant_).sentry_pose=3;
        std::get<io::SendSerialData>(send_cmd_variant_).vx = 0;
        std::get<io::SendSerialData>(send_cmd_variant_).vy = 0;
        std::get<io::SendSerialData>(send_cmd_variant_).wz = 0;
        std::get<io::SendSerialData>(send_cmd_variant_).crc16 = 0;
        std::get<std::vector<io::ReceiveSerialData>>(receive_data_variant_).reserve(1000);
    }
};

void SerialNode::plotter_debug_cmd(double& now, double vx, double vy, double wz) {
    nlohmann::json cmd_data_json;
    cmd_data_json["ts"] = now;
    cmd_data_json["cmd_vx"] = vx;
    cmd_data_json["cmd_vy"] = vy;
    cmd_data_json["cmd_wz"] = wz;
    plotter.plot(cmd_data_json);
}
void SerialNode::plotter_debug_receive(double& now) {
    nlohmann::json true_speed_data;
    true_speed_data["ts"] = now;
    true_speed_data["true_vx"] = receive_speed_.vx;
    true_speed_data["true_vy"] = receive_speed_.vy;
    true_speed_data["true_wz"] = receive_speed_.wz;
    plotter.plot(true_speed_data);
}
} // namespace io