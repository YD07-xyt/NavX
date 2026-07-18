#include <relocation_node.hpp>
int main(int argc, char **argv) {

  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>("relocation_node");

  // planner::GlobalPlanner global_planner(nh);
  relocation::RelocationNode relocation_node(node);
  rclcpp::WallRate rate(1000);
  while (rclcpp::ok()) {
    rclcpp::spin_some(node);
    rate.sleep();
  }

  rclcpp::shutdown();
  return 0;
}