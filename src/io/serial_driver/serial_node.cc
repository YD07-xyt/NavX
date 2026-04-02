#include"src/serial.h"
#include<rclcpp/rclcpp.hpp>
#include <rclcpp/utilities.hpp>
#include"serial_node.h"
//TODO
// 实现 send_status()
bool send_status() {
    std::cout << "Status sent" << std::endl;
    return true;
}

int main(int argc, char * argv[]){

    // 参数初始化
    std::string serial_name="/dev/ttyUSB0";
    int baud_rate =115200;
    int max_try=1000000;


    rclcpp::init(argc, argv);
    auto node = std::make_shared<io::SerialNode>(serial_name,baud_rate,max_try); 
    rclcpp::spin(node);
    
    rclcpp::shutdown();
    return 0;
}