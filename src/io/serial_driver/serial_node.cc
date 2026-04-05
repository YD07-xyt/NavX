#include"src/serial.h"
#include<rclcpp/rclcpp.hpp>
#include <rclcpp/utilities.hpp>
#include"serial_node.h"

int main(int argc, char * argv[]){
    rclcpp::init(argc, argv);
    
    auto serial_node = std::make_shared<rclcpp::Node>("serial_node");
    std::string serial_name;
    int baud_rate, max_try;
    // 参数初始化
    serial_node->declare_parameter<std::string>("serial_name", "/dev/ttyUSB0");
    serial_node->declare_parameter<int>("baud_rate", 115200);
    serial_node->declare_parameter<int>("max_try", 10);
        
    // 读取参数
    serial_node->get_parameter("serial_name", serial_name);
    serial_node->get_parameter("baud_rate", baud_rate);
    serial_node->get_parameter("max_try", max_try);
    

    auto node = std::make_shared<io::SerialNode>
        (serial_name,baud_rate,max_try,serial_node); 
    
    rclcpp::spin(serial_node);
    
    rclcpp::shutdown();
    return 0;
}