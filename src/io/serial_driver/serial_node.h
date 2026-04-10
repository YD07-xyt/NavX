#pragma once

#include <chrono>
#include <memory>
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
class SerialNode {
    public:
        SerialNode(const std::string& serial_name, int& baud_rate, int& max_try,const rclcpp::Node::SharedPtr node):node_(node){
            
            
            serial_driver = std::make_shared<io::SerialDriver>(serial_name,baud_rate,max_try);
            this->is_open_serial=this->serial_driver->open(serial_name, baud_rate);
            if(!is_open_serial){
                //serial_driver->reopen(serial_name,baud_rate,max_try);
            }else{
                this->cmd_sub_ = node_->create_subscription<geometry_msgs::msg::Twist>(
                "cmd_vel", 10, 
                std::bind(&SerialNode::cmd_callback, this, std::placeholders::_1));
                this->rm_data_pub_ = node_->create_publisher<rm_interfaces::msg::RmData>("rm_data", 10);
                recv_thread_ = std::thread(&SerialNode::read_callback, this);
            }
            
        }
    private:
        rclcpp::Node::SharedPtr node_;
        bool is_open_serial;
        std::shared_ptr<io::SerialDriver> serial_driver;
        SendData send_cmd;
        ReceiveData receive_data;
        rm_interfaces::msg::RmData rm_data;       
        std::thread recv_thread_;
    private:
        
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
        /*TODO: pub receive*/
        rclcpp::Publisher<rm_interfaces::msg::RmData>::SharedPtr rm_data_pub_;
        void read_callback(){
            //TODO timeout
            serial_driver->receive1(receive_data);
            rm_data.current_hp = receive_data.current_hp;
            rm_data.game_progress= receive_data.game_progress;
            rm_data.projectile_allowance_17mm =receive_data.projectile_allowance;
            rm_data_pub_->publish(rm_data);
        }
        void cmd_callback(geometry_msgs::msg::Twist::SharedPtr cmd_data){
            send_cmd.v_x =cmd_data->linear.x;
            send_cmd.v_y =cmd_data->linear.y;
            send_cmd.w_z =cmd_data->angular.z;
            RCLCPP_INFO(node_->get_logger(),"serial 发送 cmd vx: %f ,vy : %f,wz : %f",
                    send_cmd.v_x,send_cmd.v_y,send_cmd.w_z);
            serial_driver->send(send_cmd);
        };
};
}