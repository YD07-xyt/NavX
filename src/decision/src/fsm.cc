#include "../include/fsm.h"
#include <geometry_msgs/msg/detail/twist__struct.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rm_interfaces/msg/detail/aim2_nav__struct.hpp>
#include <rm_interfaces/msg/detail/nav2_aim__struct.hpp>
#include <string>
namespace decision {
FSM::FSM() : rclcpp::Node("FSM_node") {
  this->aim2nav_sub_ = this->create_subscription<rm_interfaces::msg::Aim2Nav>(
      "aim2nav", 10,
      std::bind(&FSM::aim2nav_sub_callback, this, std::placeholders::_1));
  this->cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "aft_cmd_vel", 10,
      std::bind(&FSM::cmd_sub_callback, this, std::placeholders::_1));
  this->nav2aim_pub_ =
      this->create_publisher<rm_interfaces::msg::Nav2Aim>("nav2aim", 10);
  this->goal_pub_ =
      this->create_publisher<geometry_msgs::msg::PoseStamped>("/goal_pose", 10);
  this->goal_client_ =
      rclcpp_action::create_client<NavigateToPose>(this, "/navigate_to_pose");
  // this->pub_timer_ = this->create_wall_timer(
  // std::chrono::milliseconds(100),
  // std::bind(&FSM::nav2aim_pub_callback, this));
}
void FSM::aim2nav_sub_callback(const rm_interfaces::msg::Aim2Nav &msg) {

  game_data_.current_hp = msg.current_hp;
  game_data_.game_progress = msg.game_progress;
  game_data_.projectile_allowance_17mm = msg.projectile_allowance_17mm;
  this->aim2nav_data_.emplace_back(game_data_);
  RCLCPP_INFO(this->get_logger(),
              "[Nav] Aim2NavData: current_hp:%d,game_progress: %d, "
              "projectile_allowance_17mm: %d",
              game_data_.current_hp, game_data_.game_progress,
              game_data_.projectile_allowance_17mm);
  this->sendGoal_pub_callback();
}

void FSM::nav2aim_pub_callback() {
  Nav2AimData data{1, 1, 1};
  rm_interfaces::msg::Nav2Aim nav2aim;
  nav2aim.vx = 1;
  nav2aim.vy = 1;
  nav2aim.wz = 1;
  this->nav2aim_pub_->publish(nav2aim);
  RCLCPP_INFO(this->get_logger(), "nav2aim : vx:%f,vy:%f,wz:%f", nav2aim.vx,
              nav2aim.vy, nav2aim.wz);
}

void FSM::cmd_sub_callback(const geometry_msgs::msg::Twist &msg) {
  rm_interfaces::msg::Nav2Aim nav2aim;
  nav2aim.vx = msg.linear.x;
  nav2aim.vy = msg.linear.y;
  nav2aim.wz = msg.angular.z;
  this->nav2aim_pub_->publish(nav2aim);
  RCLCPP_INFO(this->get_logger(), "nav2aim : vx:%f,vy:%f,wz:%f", nav2aim.vx,
              nav2aim.vy, nav2aim.wz);
}

void FSM::send_pub_Goal(Point goal_point, std::string map_tf_name) {
  auto msg = geometry_msgs::msg::PoseStamped();

  // 设置时间戳和坐标系
  msg.header.stamp = this->now();
  msg.header.frame_id = map_tf_name; //!!!

  // 设置位置
  msg.pose.position.x = goal_point.x;
  msg.pose.position.y = goal_point.y;
  msg.pose.position.z = 0.0;

  // 设置方向（四元数）
  msg.pose.orientation.x = 0.0;
  msg.pose.orientation.y = 0.0;
  msg.pose.orientation.z = 0.0;
  msg.pose.orientation.w = 1.0;

  goal_pub_->publish(msg);
  RCLCPP_INFO(this->get_logger(), "Published goal: (%.2f, %.2f, %.2f rad)",
              goal_point.x, goal_point.y, goal_point.yaw);
}

void FSM::sendGoal_pub_callback() {
  if (game_data_.game_progress == 1) {
    if (game_data_.current_hp <= 100 ||
        game_data_.projectile_allowance_17mm <= 50) {
      RCLCPP_INFO(this->get_logger(), "robot start is bad,go home");
      send_pub_Goal(goal_point_sum.home, this->map_tf_name_);
    };
    send_pub_Goal(goal_point_sum.Patrol1, this->map_tf_name_);
  } else {
    RCLCPP_INFO(this->get_logger(), "game is not start");
  }
}

void FSM::sendGoal_action_callback() {
  if (game_data_.game_progress == 1) {

    if (game_data_.current_hp <= 100 ||
        game_data_.projectile_allowance_17mm <= 50) {
      RCLCPP_INFO(this->get_logger(), "robot start is bad,go home");
      this->send_action_goal(goal_point_sum.home, this->map_tf_name_);
      if (this->is_goal_accepted == false) {
        resend_action_goal(goal_point_sum.home, this->map_tf_name_);
      }
    };
    if (this->is_succeeded == true) {
      this->is_succeeded = false;
      this->send_action_goal(goal_point_sum.Patrol1, this->map_tf_name_);
      if (this->is_goal_accepted == false) {
        resend_action_goal(goal_point_sum.home, this->map_tf_name_);
      }
      if (this->is_succeeded == true) {
        this->is_succeeded = false;
        std::this_thread::sleep_for(std::chrono::seconds(5));
        this->send_action_goal(goal_point_sum.Patrol2, this->map_tf_name_);
        if (this->is_goal_accepted == false) {
          resend_action_goal(goal_point_sum.home, this->map_tf_name_);
        }
      }
    }
  } else {
    RCLCPP_INFO(this->get_logger(), "game is not start");
  }
};
void FSM::resend_action_goal(Point point, std::string map_tf_name) {
  int i = 0;
  this->send_action_goal(point, map_tf_name);
};
void FSM::send_action_goal(Point point, std::string map_tf_name) {
  // 等待服务端就绪
  if (!goal_client_->wait_for_action_server(std::chrono::seconds(10))) {
    RCLCPP_ERROR(get_logger(), "Action server not available");
    return;
  }

  // 构建目标消息
  auto goal_msg = NavigateToPose::Goal();
  goal_msg.pose.header.frame_id = map_tf_name;
  goal_msg.pose.header.stamp = now();
  goal_msg.pose.pose.position.x = point.x;
  goal_msg.pose.pose.position.y = point.y;
  goal_msg.pose.pose.position.z = 0.0;

  // 将偏航角转换为四元数
  goal_msg.pose.pose.orientation.x = 0.0;
  goal_msg.pose.pose.orientation.y = 0.0;
  goal_msg.pose.pose.orientation.z = 0.0;
  goal_msg.pose.pose.orientation.w = 1.0;

  goal_msg.behavior_tree = ""; // 使用默认行为树

  // 设置回调选项
  auto send_goal_options =
      rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
  send_goal_options.goal_response_callback =
      std::bind(&FSM::goal_response_callback, this, std::placeholders::_1);
  send_goal_options.feedback_callback =
      std::bind(&FSM::feedback_callback, this, std::placeholders::_1,
                std::placeholders::_2);
  send_goal_options.result_callback =
      std::bind(&FSM::result_callback, this, std::placeholders::_1);

  // 异步发送目标
  this->goal_client_->async_send_goal(goal_msg, send_goal_options);
  RCLCPP_INFO(get_logger(), "Sending goal: (%.2f, %.2f, %.2f rad)", point.x,
              point.y, point.yaw);
}

void FSM::goal_response_callback(const GoalHandle::SharedPtr &goal_handle) {
  if (!goal_handle) {
    RCLCPP_ERROR(get_logger(), "Goal was rejected");
    this->is_goal_accepted = false;
  } else {
    RCLCPP_INFO(get_logger(), "Goal accepted");
    this->is_goal_accepted = true;
  }
}

// 反馈回调
void FSM::feedback_callback(
    GoalHandle::SharedPtr,
    const std::shared_ptr<const NavigateToPose::Feedback> feedback) {
  RCLCPP_INFO(get_logger(), "Distance remaining: %.2f, recoveries: %d",
              feedback->distance_remaining, feedback->number_of_recoveries);
}

// 结果回调
void FSM::result_callback(const GoalHandle::WrappedResult &result) {
  switch (result.code) {
  case rclcpp_action::ResultCode::SUCCEEDED:
    RCLCPP_INFO(get_logger(), "Goal succeeded!");
    this->is_succeeded = true;
    break;
  case rclcpp_action::ResultCode::ABORTED:
    this->is_succeeded = false;
    RCLCPP_ERROR(this->get_logger(), "Unknown result code: %d",
                 static_cast<int>(result.code));
    break;
  case rclcpp_action::ResultCode::CANCELED:
    this->is_succeeded = false;
    RCLCPP_INFO(get_logger(), "Goal canceled");
    break;
  default:
    this->is_succeeded = false;
    RCLCPP_ERROR(get_logger(), "Unknown result code");
  }
  rclcpp::shutdown();
}
} // namespace decision