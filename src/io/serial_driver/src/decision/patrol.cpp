#include "patrol.h"
#include <chrono>

namespace decision {

// 选择目标点
Point Patrol::selectTarget() {
  // 正常巡逻
  switch (current_patrol_index_) {
  case 0:
    return goal_point_sum_.Patrol1;
  case 1:
    return goal_point_sum_.Patrol2;
  case 2: 
    return goal_point_sum_.Patrol3;
  case 3:
    return goal_point_sum_.Patrol4;
  default:
    current_patrol_index_ = 0;
    RCLCPP_INFO(node_->get_logger()," Patrol::selectTarget()  default");
    return goal_point_sum_.Patrol1;
  }
}
void Patrol::advancePatrolIndex() {
  //TODO: 导航耗时和等待时间分离
  auto waitStartTime = std::chrono::steady_clock::now();
  
  auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                      std::chrono::steady_clock::now() - waitStartTime)
                      .count();
  elapsed=6.0;

  switch (current_patrol_index_) {
  case 0:
    if (elapsed >= patrol_wait_time_.wait_point1_time) {

      current_patrol_index_ = 1;
      
      RCLCPP_INFO(node_->get_logger(), "Patrol1 -> Patrol2");
    }
    break;

  case 1:
    if (elapsed >= patrol_wait_time_.wait_point2_time) {
      current_patrol_index_ = 2;
      RCLCPP_INFO(node_->get_logger(), "Patrol2 -> Patrol3");
      break;
    }
  case 2:
    if (elapsed >= patrol_wait_time_.wait_point3_time) {
      current_patrol_index_ = 3;
      RCLCPP_INFO(node_->get_logger(), "Patrol3 -> Patrol4");
      break;
    }
  case 3:
    if (elapsed >= patrol_wait_time_.wait_point4_time) {
      current_patrol_index_ = 0;
      RCLCPP_INFO(node_->get_logger(), "Patrol4 -> Patrol1 (loop)");
      break;
    }
  }
}
} // namespace decision