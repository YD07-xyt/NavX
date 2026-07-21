// #include "../rm_decision/include/action/topics2blackboard.hpp"
#include "rm_decision.hpp"
#include <thread>
using namespace std::chrono_literals;

int main(int argc, char *argv[]) {

  // spdlog::set_level(spdlog::level::debug);

  auto start = std::chrono::steady_clock::now();
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>("test_node");
  bt::DecisionConfig decision_config;
  decision_config.tree_xml_file =
      "/home/xyt/nav_now/src/serial_driver/src/rm_decision/behavior_tree/test/"
      "main_tree.xml";
  decision_config.tree_node_model_export_path =
      "/home/xyt/nav_now/src/serial_driver/src/rm_decision/behavior_tree/test/"
      "generated_models.xml";
  decision_config.map_tf_name = "map";
  bt::RmDecision rm_decision(node, decision_config);
  bt::Topics2Blackboard::GameData game_data;
  int tick_count = 0;
  game_data.current_hp = 400;
  game_data.projectile_allowance = 200;
  game_data.is_game_start = false;
  game_data.current_enemy_outpost_hp = 400;
  game_data.game_time = 0.0;
  bool game_started = false;
  bool hp_decreased = false;
  bool hp_restored = false; // 新增标志：是否已恢复血量
  // 在 main 开头声明新标志
  bool outpost_hp_set_30 = false;
  bool outpost_hp_set_300 = false;
  rm_decision.setBlackboardValue("nav_model", "spinning");
  rm_decision.setBlackboardValue("uphill_done", false);
  rm_decision.setBlackboardValue("downhill_done", true);
  while (rclcpp::ok()) {
    auto now = std::chrono::steady_clock::now();
    auto elapsed =
        std::chrono::duration_cast<std::chrono::seconds>(now - start).count();

    // 原有：第 3 秒游戏开始
    if (!game_started && elapsed >= 3) {
      game_data.is_game_start = true;
      spdlog::info("[test] game started at {}s", elapsed);
      game_started = true;
    }

    // 新增：第 10 秒设置前哨站血量为 30
    if (!outpost_hp_set_30 && elapsed >= 10) {
      game_data.current_enemy_outpost_hp = 30;
      spdlog::info("[test] set current_enemy_outpost_hp to 30 at {}s", elapsed);
      outpost_hp_set_30 = true;
    }

    // 新增：第 15 秒恢复前哨站血量为 300
    // if (!outpost_hp_set_300 && elapsed >= 20) {
    //   game_data.current_enemy_outpost_hp = 300;
    //   spdlog::info("[test] set current_enemy_outpost_hp to 300 at {}s",
    //                elapsed);
    //   outpost_hp_set_300 = true;
    // }

    if (!hp_decreased && elapsed >= 30) {
      game_data.current_hp = 20;
      spdlog::info("[test] set current_hp to 20 at {}s", elapsed);
      hp_decreased = true;
    }

    // 原有：第 35 秒血量恢复
    if (!hp_restored && elapsed >= 45) {
      game_data.current_hp = 360;
      spdlog::info("[test] restore current_hp to 200 at {}s", elapsed);
      hp_restored = true;
    }
    auto time = 420.0 - elapsed;
    game_data.game_time = static_cast<int>(time);
    spdlog::debug("game_data.game_time:{}", game_data.game_time);
    tick_count++;

    rclcpp::spin_some(node);

    auto nav_model_before =
        rm_decision.getBlackboardValue<std::string>("nav_model");
    if (nav_model_before.has_value()) {
        spdlog::debug("before tick: nav_model = {}", nav_model_before.value());
      
    }

    rm_decision.tree_tick(game_data, std::chrono::milliseconds(1));
  }
  rclcpp::shutdown();
  return 0;
}