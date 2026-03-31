#include <rclcpp/rclcpp.hpp>

#include "rog_map_ros/rog_map_ros2.hpp"
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    
    // 创建节点
    auto node = std::make_shared<rclcpp::Node>("rog_map_node");
    
    // 配置文件路径
    std::string config_path = node->declare_parameter<std::string>("config_path", "config/rog_map.yaml");
    
    // 创建ROGMapROS实例
    auto rog_map = std::make_shared<rog_map::ROGMapROS>(node, config_path);
    
    // 运行
    rclcpp::spin(node);
    rclcpp::shutdown();
    
    return 0;
}