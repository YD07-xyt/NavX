#include "serial_node.h"
#include "src/decision.h"
#include "src/serial.h"
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/utilities.hpp>

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  auto serial_node = std::make_shared<rclcpp::Node>("serial_node");
  std::string serial_name;
  int baud_rate, max_try;
  decision::GoalPoint temp_goal_point;
  bool is_decision;
  // 参数初始化
  serial_node->declare_parameter<std::string>("serial_name", "/dev/ttyUSB0");
  serial_node->declare_parameter<int>("baud_rate", 115200);
  serial_node->declare_parameter<int>("max_try", 10);

  serial_node->declare_parameter<std::string>("socket_send_name",
                                              "/tmp/serial_mux.sock");
  serial_node->declare_parameter<std::string>("socket_receive_name",
                                              "/tmp/serial_nav.sock");

  // 声明参数（带默认值）
  serial_node->declare_parameter("patrol1.x", 1.0);
  serial_node->declare_parameter("patrol1.y", 1.0);
  serial_node->declare_parameter("patrol1.yaw", 1.0);

  serial_node->declare_parameter("patrol2.x", 2.0);
  serial_node->declare_parameter("patrol2.y", 2.0);
  serial_node->declare_parameter("patrol2.yaw", 2.0);

  serial_node->declare_parameter("home.x", 0.229);
  serial_node->declare_parameter("home.y", 0.807);
  serial_node->declare_parameter("home.yaw", 0.0);
  serial_node->declare_parameter("is_decision", true);

  serial_node->get_parameter("serial_name", serial_name);
  serial_node->get_parameter("baud_rate", baud_rate);
  serial_node->get_parameter("max_try", max_try);
  
  auto node = std::make_shared<io::SerialNode>(serial_name, baud_rate, max_try,
                                               serial_node);

  // 读取参数
  serial_node->get_parameter("is_decision", node->is_decision_);
  serial_node->get_parameter("socket_send_name", node->socket_send_name);
  serial_node->get_parameter("socket_receive_name", node->socket_receive_name);
  temp_goal_point.Patrol1.x =
      serial_node->get_parameter("patrol1.x").as_double();
  temp_goal_point.Patrol1.y =
      serial_node->get_parameter("patrol1.y").as_double();
  temp_goal_point.Patrol1.yaw =
      serial_node->get_parameter("patrol1.yaw").as_double();

  temp_goal_point.Patrol2.x =
      serial_node->get_parameter("patrol2.x").as_double();
  temp_goal_point.Patrol2.y =
      serial_node->get_parameter("patrol2.y").as_double();
  temp_goal_point.Patrol2.yaw =
      serial_node->get_parameter("patrol2.yaw").as_double();

  temp_goal_point.home.x = serial_node->get_parameter("home.x").as_double();
  temp_goal_point.home.y = serial_node->get_parameter("home.y").as_double();
  temp_goal_point.home.yaw = serial_node->get_parameter("home.yaw").as_double();

  node->init_goal(temp_goal_point);

  rclcpp::spin(serial_node);

  rclcpp::shutdown();
  return 0;
}