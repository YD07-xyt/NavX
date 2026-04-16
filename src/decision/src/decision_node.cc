#include "../include/fsm.h"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char** argv)
{
    // 初始化ROS 2
    rclcpp::init(argc, argv);
    
    // 创建节点共享指针
    auto fsm = std::make_shared<decision::FSM>();
    
    // 旋转节点（处理回调）
    rclcpp::spin(fsm);
    
    // 关闭ROS 2
    rclcpp::shutdown();
    
    return 0;
}