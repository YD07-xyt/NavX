#pragma once

#include "behaviortree_cpp/behavior_tree.h"
#include "behaviortree_cpp/json_export.h"
#include "config.hpp"
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

namespace BT {

template <>
[[nodiscard]] inline bt::Point convertFromString<bt::Point>(std::string_view str) {
  const auto parts = BT::splitString(str, ',');
  if (parts.size() != 3) {
    throw BT::RuntimeError("invalid input");
  }

  bt::Point nav_point(0.0, 0.0, 0.0);
  nav_point.x = convertFromString<double>(parts[0]);
  nav_point.y = convertFromString<double>(parts[1]);
  nav_point.yaw = convertFromString<double>(parts[2]);
  return nav_point;
}

// inline void NavPointToJson(nlohmann::json &dest, const bt::Point &point) {
//   dest["x"] = point.x;
//   dest["y"] = point.y;
//   dest["yaw"] = point.yaw;
// }


} // namespace BT
