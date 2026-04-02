#pragma once

#include <chrono>
#include <rclcpp/publisher.hpp>
#include<rclcpp/rclcpp.hpp>
#include<rclcpp/subscription.hpp>
#include<geometry_msgs/msg/twist.hpp>

#include <rclcpp/timer.hpp>
#include <rm_interfaces/msg/detail/rm_data__struct.hpp>
#include<rm_interfaces/msg/rm_data.hpp>

#include"src/packet_typedef.h"
#include "src/serial.h"
namespace io {
class SerialNode : public rclcpp::Node {
    public:
        SerialNode(const std::string& serial_name, int& baud_rate, int& max_try): Node("serial_node"){
            this->is_open_serial=this->serial_driver.open(serial_name, baud_rate);
            if(is_open_serial){
                serial_driver.reopen(serial_name,baud_rate,max_try);
            }
            this->cmd_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "aft_cmd_vel", 10, 
            std::bind(&SerialNode::cmd_callback, this, std::placeholders::_1));
            io_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(1),
            [this]() {
                read_callback();  // 处理异步事件
            }
        );
            
        }
    private:
        rclcpp::TimerBase::SharedPtr io_timer_;
        bool is_open_serial;
        io::SerialDriver serial_driver;
        SendData send_cmd;
        ReceiveData receive_data;
        rm_interfaces::msg::RmData rm_data;       
    private:
        
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
        /*TODO: pub receive*/
        rclcpp::Publisher<rm_interfaces::msg::RmData>::SharedPtr rm_data_pub_;
        void read_callback(){
            serial_driver.receive(receive_data);
            rm_data.current_hp = receive_data.current_hp;
            rm_data.game_progress= receive_data.game_progress;
            rm_data.projectile_allowance_17mm =receive_data.projectile_allowance;
            rm_data_pub_->publish(rm_data);
        }
        void cmd_callback(geometry_msgs::msg::Twist::SharedPtr cmd_data){
            send_cmd.v_x =cmd_data->linear.x;
            send_cmd.v_y =cmd_data->linear.y;
            send_cmd.w_z =cmd_data->angular.z;
            serial_driver.send(send_cmd);
        };

};
}