#pragma once

#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include "behaviortree_cpp/json_export.h"
namespace bt {
// enum GameState {
//   idle, //一般未开始
//   running,
//   end,
// };

// enum SentryPosture {
//   attack = 1,
//   defense = 2,
//   move = 3,
// };

struct DecisionConfig {
  std::string tree_xml_file="";
  std::string tree_node_model_export_path="";
  std::string pub_goal_topic_name="/goal_pose";
  std::string nav2_state_topic_name="/navigate_to_pose/_action/status";
  std::string map_tf_name="";
  //std::string odom_sub_topic="";
  int send_goal_timeout=100;
};


struct Point {
  double x=0.0;
  double y=0.0;
  double yaw=0.0;
  // 默认构造函数（必须提供）
  Point() =default;
  Point(double x, double y, double yaw) : x(x), y(y), yaw(yaw){};
};
inline void to_json(nlohmann::json& j, const bt::Point& p) {
    j["x"] = p.x; j["y"] = p.y; j["yaw"] = p.yaw;
}
inline void from_json(const nlohmann::json& j, bt::Point& p) {
    j.at("x").get_to(p.x);
    j.at("y").get_to(p.y);
    j.at("yaw").get_to(p.yaw);
}
} // namespace bt