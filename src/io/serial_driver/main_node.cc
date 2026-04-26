#include "src/decision/decision.h"
#include "src/ros2/serial_node.h"
#include "src/serial/serial.h"
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/utilities.hpp>
#include <string>

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  auto serial_node = std::make_shared<rclcpp::Node>("serial_node");
  std::string serial_name;
  int baud_rate, max_try;
  decision::GoalPoint temp_goal_point;
  std::string socket_send_name;
  std::string socket_receive_name;
  io::SendingMethod sending_method;
  std::string sending_method_name;
  decision::PatrolWaitTime temp_patrol_wait_time;
  decision::StateIsGoHome temp_state_is_go_home;
  // 参数初始化
  serial_node->declare_parameter<std::string>("serial_name", "/dev/ttyUSB0");
  serial_node->declare_parameter<int>("baud_rate", 115200);
  serial_node->declare_parameter<int>("max_try", 10);

  serial_node->declare_parameter<std::string>("socket_send_name",
                                              "/tmp/serial_mux.sock");
  serial_node->declare_parameter<std::string>("socket_receive_name",
                                              "/tmp/serial_nav.sock");
  serial_node->declare_parameter<std::string>("sending_method", "socket");

  // 声明参数（带默认值）
  serial_node->declare_parameter("patrol1.x", 1.0);
  serial_node->declare_parameter("patrol1.y", 1.0);
  serial_node->declare_parameter("patrol1.yaw", 1.0);
  serial_node->declare_parameter("patrol1.wait_time", 10);

  serial_node->declare_parameter("patrol2.x", 2.0);
  serial_node->declare_parameter("patrol2.y", 2.0);
  serial_node->declare_parameter("patrol2.yaw", 2.0);
  serial_node->declare_parameter("patrol2.wait_time", 10);

  serial_node->declare_parameter("patrol3.x", 1.0);
  serial_node->declare_parameter("patrol3.y", 1.0);
  serial_node->declare_parameter("patrol3.yaw", 1.0);
  serial_node->declare_parameter("patrol3.wait_time", 10);

  serial_node->declare_parameter("patrol4.x", 2.0);
  serial_node->declare_parameter("patrol4.y", 2.0);
  serial_node->declare_parameter("patrol4.yaw", 2.0);
  serial_node->declare_parameter("patrol4.wait_time", 10);

  serial_node->declare_parameter("HitOutpost.x", 2.0);
  serial_node->declare_parameter("HitOutpost.y", 2.0);
  serial_node->declare_parameter("HitOutpost.yaw", 2.0);

  serial_node->declare_parameter("go_home_hp", 2.0);
  serial_node->declare_parameter("go_home_projectile_allowance", 2.0);
  serial_node->declare_parameter("become_home_hp", 2.0);
  serial_node->declare_parameter("become_home_projectile_allowance", 2.0);

  serial_node->declare_parameter("is_decision", true);

  serial_node->get_parameter("serial_name", serial_name);
  serial_node->get_parameter("baud_rate", baud_rate);
  serial_node->get_parameter("max_try", max_try);

  serial_node->get_parameter("socket_send_name", socket_send_name);
  serial_node->get_parameter("socket_receive_name", socket_receive_name);
  serial_node->get_parameter("sending_method", sending_method_name);
  if (sending_method_name == "socket") {
    RCLCPP_INFO(serial_node->get_logger(), "Using socket for communication");
    sending_method = io::SendingMethod::socket;
  } else {
    RCLCPP_INFO(serial_node->get_logger(), "Using serial for communication");
    sending_method = io::SendingMethod::serial;
  }
  auto node = std::make_shared<io::SerialNode>(
      serial_name, baud_rate, max_try, serial_node, socket_send_name,
      socket_receive_name, sending_method);

  // 读取参数
  serial_node->get_parameter("is_decision", node->is_decision_);

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

  temp_goal_point.Patrol3.x =
      serial_node->get_parameter("patrol3.x").as_double();
  temp_goal_point.Patrol3.y =
      serial_node->get_parameter("patrol3.y").as_double();
  temp_goal_point.Patrol3.yaw =
      serial_node->get_parameter("patrol3.yaw").as_double();

  temp_goal_point.Patrol4.x =
      serial_node->get_parameter("patrol4.x").as_double();
  temp_goal_point.Patrol4.y =
      serial_node->get_parameter("patrol4.y").as_double();
  temp_goal_point.Patrol4.yaw =
      serial_node->get_parameter("patrol4.yaw").as_double();

  temp_goal_point.HitOutpost.x =
      serial_node->get_parameter("HitOutpost.x").as_double();
  temp_goal_point.HitOutpost.y =
      serial_node->get_parameter("HitOutpost.y").as_double();
  temp_goal_point.HitOutpost.yaw =
      serial_node->get_parameter("HitOutpost.yaw").as_double();
  temp_goal_point.home.x = serial_node->get_parameter("home.x").as_double();
  temp_goal_point.home.y = serial_node->get_parameter("home.y").as_double();
  temp_goal_point.home.yaw = serial_node->get_parameter("home.yaw").as_double();
  temp_patrol_wait_time.wait_point1_time =
      serial_node->get_parameter("patrol1.wait_time").as_double();
  temp_patrol_wait_time.wait_point2_time =
      serial_node->get_parameter("patrol2.wait_time").as_double();
  temp_patrol_wait_time.wait_point3_time =
      serial_node->get_parameter("patrol3.wait_time").as_double();
  temp_patrol_wait_time.wait_point4_time =
      serial_node->get_parameter("patrol4.wait_time").as_double();
  temp_state_is_go_home.become_home_hp =
      serial_node->get_parameter("become_home_hp").as_int();
  temp_state_is_go_home.become_home_projectile_allowance =
      serial_node->get_parameter("become_home_projectile_allowance").as_int();
  temp_state_is_go_home.go_home_projectile_allowance =
      serial_node->get_parameter("go_home_projectile_allowance").as_int();
  temp_state_is_go_home.go_home_hp =
      serial_node->get_parameter("go_home_hp").as_int();
  node->init_goal(temp_goal_point, temp_patrol_wait_time,
                  temp_state_is_go_home);

  rclcpp::spin(serial_node);

  rclcpp::shutdown();
  return 0;
}