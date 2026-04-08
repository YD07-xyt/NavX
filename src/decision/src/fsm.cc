#include "../include/fsm.h"
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rm_interfaces/msg/detail/aim2_nav__struct.hpp>
#include <rm_interfaces/msg/detail/nav2_aim__struct.hpp>
namespace decision {
FSM::FSM() : rclcpp::Node("FSM_node") {
  this->aim2nav_sub_ = this->create_subscription<rm_interfaces::msg::Aim2Nav>(
      "aim2nav", 10,
      std::bind(&FSM::aim2nav_sub_callback, this, std::placeholders::_1));
  this->nav2aim_pub_ =
      this->create_publisher<rm_interfaces::msg::Nav2Aim>("nav2aim", 10);
  this->pub_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(100),
    std::bind(&FSM::nav2aim_pub_callback, this)
);
}
void FSM::aim2nav_sub_callback(const rm_interfaces::msg::Aim2Nav &msg) {
  Aim2NavData data;
  data.current_hp = msg.current_hp;
  data.game_progress = msg.game_progress;
  data.projectile_allowance_17mm = msg.projectile_allowance_17mm;
  this->aim2nav_data_.emplace_back(data);
  RCLCPP_INFO(this->get_logger(),
              "[Nav] Aim2NavData: current_hp:%d,game_progress: %d, "
              "projectile_allowance_17mm: %d",
              data.current_hp, data.game_progress,
              data.projectile_allowance_17mm);
    
}

void FSM::nav2aim_pub_callback(){
    Nav2AimData data{1,1,1};
    rm_interfaces::msg::Nav2Aim nav2aim;
    nav2aim.vx=1;
    nav2aim.vy=1;
    nav2aim.wz=1;
    this->nav2aim_pub_->publish(nav2aim);
    RCLCPP_INFO(this->get_logger(),"nav2aim : vx:%f,vy:%f,wz:%f",nav2aim.vx,nav2aim.vy,nav2aim.wz);
}
} // namespace decision