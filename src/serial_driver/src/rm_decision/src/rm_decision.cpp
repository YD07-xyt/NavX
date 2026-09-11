#include"rm_decision/rm_decision.hpp"
#include"rm_decision/api.hpp"
#include "action/async_sleep.hpp"
#include "action/nav2pose.hpp"
#include "action/switch_nav_model.hpp"
#include "condition/check_blackboard_bool.hpp"
#include "condition/is_enemy_outpost_destroyed.hpp"
#include "condition/is_fort_occupied.hpp"
#include "condition/is_game_start.hpp"
#include "condition/is_game_time_exceeded.hpp"
#include "condition/is_gmae_time_less.hpp"
#include "condition/is_go_home.hpp"
#include "condition/is_robot_health.hpp"
#include "condition/set_blackboard_bool.hpp"
#include <fstream>
namespace bt {
  RmDecision::RmDecision(bt::DecisionConfig decision_config)
      : config_(decision_config) {

    unsigned int groot_port = 5556;
    setting_tree_custom_node();
    // TODO:
    BT::RegisterJsonDefinition<bt::Point>();

    std::string xml_models = BT::writeTreeNodesModelXML(factory_);
    // save to file
    std::ofstream file(config_.tree_node_model_export_path);
    file << xml_models;
    file.close();
    logger::info(logger, "Generated XML file");
    tree_ = factory_.createTreeFromFile(config_.tree_xml_file.c_str());
    tree_blackboard_ = tree_.rootBlackboard();
    intput_blackboard_ = std::make_shared<Intputblackboard>(tree_blackboard_);
    groot2publisher_ptr_ =
        std::make_unique<BT::Groot2Publisher>(tree_, groot_port);
  };
    auto RmDecision::tree_tick(Game2Decision &game_data,Nav2Decision &nav_data,
                 std::optional<std::chrono::system_clock::duration> timeout)->void {
    intput_blackboard_->set_data(game_data, nav_data);
    tree_.tickOnce();
    if (timeout.has_value()) {
      tree_.sleep(timeout.value());
    }
  }
  auto RmDecision::get_data()->std::pair<std::optional<Decision2Game>, std::optional<Decision2Nav>>{
      return output_blackboard_->get_data();
  }
    void RmDecision::setting_tree_custom_node() {
    // factory_.registerNodeType<BT::>("Wait");
    register_enum(factory_);
    factory_.registerBuilder<bt::Nav2Pose>(
        "Nav2Pose",
        [this](const std::string &name, const BT::NodeConfig &config) {
          return std::make_unique<bt::Nav2Pose>(name, config);
        });
    factory_.registerNodeType<bt::SwitchNavModel>("SwitchNavModel");
    factory_.registerNodeType<bt::AsyncSleep>("AsyncSleep");  // 新增这一行
    // factory_.registerNodeType<bt::Topics2Blackboard>("Topics2Blackboard");
    factory_.registerNodeType<bt::IsGameTimeExceeded>("IsGameTimeExceeded");
    factory_.registerNodeType<bt::IsGameTimeLess>("IsGameTimeLess");
    factory_.registerNodeType<bt::IsFortOccupied>("IsFortOccupied");
    factory_.registerNodeType<bt::IsEnemyOutpostDestroyed>(
        "IsEnemyOutpostDestroyed");
    factory_.registerNodeType<bt::IsGameStart>("IsGameStart");
    factory_.registerNodeType<bt::IsGoHome>("IsGoHome");
    factory_.registerNodeType<bt::IsRobotHealth>("IsRobotHealth");
    factory_.registerNodeType<bt::CheckBlackboardBool>("CheckBlackboardBool");
    factory_.registerNodeType<bt::SetBlackboardBool>("SetBlackboardBool");
    spdlog::info("Loaded all custom nodes");
  };
}