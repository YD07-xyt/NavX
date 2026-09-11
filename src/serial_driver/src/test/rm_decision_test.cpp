// #include "../rm_decision/include/action/topics2blackboard.hpp"
#include "rm_decision/rm_decision.hpp"
#include "test_node/test_node.hpp"
#include <rclcpp/executors.hpp>
#include <thread>
using namespace std::chrono_literals;

int main(int argc, char* argv[]) {
    // spdlog::set_level(spdlog::level::debug);

    auto start = std::chrono::steady_clock::now();
    rclcpp::init(argc, argv);

    auto node = std::make_shared<rclcpp::Node>("test_node");
    bt::DecisionConfig decision_config;
    decision_config.tree_xml_file = "/home/xyt/map/src/serial_driver/src/rm_decision/behavior_tree/rmuc2026.xml";
    decision_config.tree_node_model_export_path =
        "/home/xyt/map/src/serial_driver/src/rm_decision/behavior_tree/test/generated_models.xml";
    decision_config.map_tf_name = "world";
    test::TestNode test_node(node, decision_config);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}