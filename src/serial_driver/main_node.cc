#include "ros2/node.h"
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/utilities.hpp>
#include <spdlog/spdlog.h>
#include <string>
int main(int argc, char *argv[]) {
  rclcpp::init(argc,argv);
  auto node=rclcpp::Node::make_shared("ma_node");
  ros2::Ros2config config;
   if (!load_config("/home/xyt/map/src/serial_driver/config/param.yaml", config)) {
    return -1;
   }
  ros2::SerialNode serial_node(config,node);
  
  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;

}
