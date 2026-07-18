#pragma once
#include "action/async_sleep.hpp"
#include "action/nav2pose.hpp"
#include "action/topics2blackboard.hpp"
#include "condition/check_blackboard_bool.hpp"
#include "condition/is_enemy_outpost_destroyed.hpp"
#include "condition/is_game_start.hpp"
#include "condition/is_game_time_exceeded.hpp"
#include "condition/is_gmae_time_less.hpp"
#include "condition/is_go_home.hpp"
#include "condition/is_robot_health.hpp"
#include "condition/is_success_nav.hpp"
#include "condition/set_blackboard_bool.hpp"
#include "config.hpp"
#include "custume_types.hpp"
#include "rm_decision/include/action/switch_nav_model.hpp"

#include <behaviortree_cpp/actions/sleep_node.h>
#include <behaviortree_cpp/behavior_tree.h>
#include <behaviortree_cpp/blackboard.h>
#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/json_export.h>
#include <behaviortree_cpp/loggers/groot2_publisher.h>
#include <behaviortree_cpp/xml_parsing.h>
#include <fstream>
#include <memory>
#include <optional>

namespace bt {
class RmDecision {
public:
  RmDecision(const rclcpp::Node::SharedPtr node,
             bt::DecisionConfig decision_config)
      : node_(node), config_(decision_config),
        node_with_param_(NodeWithParam(node, decision_config)) {

    unsigned int groot_port = 5556;
    setting_tree_custom_node();
    // TODO:
    BT::RegisterJsonDefinition<bt::Point>();

    std::string xml_models = BT::writeTreeNodesModelXML(factory_);
    // save to file
    std::ofstream file(config_.tree_node_model_export_path);
    file << xml_models;
    file.close();
    spdlog::info("Generated XML file");
    tree_ = factory_.createTreeFromFile(config_.tree_xml_file.c_str());
    tree_blackboard_ = tree_.rootBlackboard();
    topics2blackboard_ = std::make_shared<Topics2Blackboard>(tree_blackboard_);
    groot2publisher_ptr_ =
        std::make_unique<BT::Groot2Publisher>(tree_, groot_port);
  };

  void tree_tick(Topics2Blackboard::GameData &game_data,
                 std::optional<std::chrono::system_clock::duration> timeout) {
    topics2blackboard_->TopicSetCallback(game_data);
    tree_.tickOnce();
    if (timeout.has_value()) {
      tree_.sleep(timeout.value());
    }
  }

  template <typename T>
  std::optional<T> getBlackboardValue(const std::string &key) const {
    return tree_blackboard_->get<T>(key);
  }
  template <typename T>
  void setBlackboardValue(const std::string &key, const T& value) {
      tree_blackboard_->set(key, value);
  }
private:
  void setting_tree_custom_node() {
    // factory_.registerNodeType<BT::>("Wait");

    factory_.registerBuilder<bt::Nav2Pose>(
        "Nav2Pose",
        [this](const std::string &name, const BT::NodeConfig &config) {
          // 确保 node_with_param_ 有值（已在构造函数中初始化）
          return std::make_unique<bt::Nav2Pose>(name, config, node_with_param_);
        });
    factory_.registerNodeType<bt::SwitchNavModel>("SwitchNavModel");
    factory_.registerNodeType<bt::AsyncSleep>("AsyncSleep");  // 新增这一行
    // factory_.registerNodeType<bt::Topics2Blackboard>("Topics2Blackboard");
    factory_.registerNodeType<bt::IsGameTimeExceeded>("IsGameTimeExceeded");
    factory_.registerNodeType<bt::IsGameTimeLess>("IsGameTimeLess");
    factory_.registerNodeType<bt::IsEnemyOutpostDestroyed>(
        "IsEnemyOutpostDestroyed");
    factory_.registerNodeType<bt::IsGameStart>("IsGameStart");
    factory_.registerNodeType<bt::IsGoHome>("IsGoHome");
    factory_.registerNodeType<bt::IsRobotHealth>("IsRobotHealth");
    factory_.registerNodeType<bt::IsSuccessNav>("IsSuccessNav");
    factory_.registerNodeType<bt::CheckBlackboardBool>("CheckBlackboardBool");
    factory_.registerNodeType<bt::SetBlackboardBool>("SetBlackboardBool");
    spdlog::info("Loaded all custom nodes");
  };

private:
  rclcpp::Node::SharedPtr node_;
  bt::DecisionConfig config_;
  std::optional<bt::NodeWithParam> node_with_param_;
  std::shared_ptr<Topics2Blackboard> topics2blackboard_;

private:
  BT::Tree tree_;
  BT::Blackboard::Ptr tree_blackboard_;
  std::shared_ptr<BT::Groot2Publisher> groot2publisher_ptr_;
  BT::BehaviorTreeFactory factory_;
};
} // namespace bt
