#include "test_node.hpp"
#include "rm_decision/api.hpp"
#include <geometry_msgs/msg/detail/twist__struct.hpp>
#include <spdlog/spdlog.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs/tf2_geometry_msgs.hpp>
namespace test {
void TestNode::nav_feedback_callback(const std_msgs::msg::Int16::SharedPtr msg) {
    if (msg->data == 0) {
        nav_data.nav_state = bt::NavState::IDLE;
    } else if (msg->data == 1) {
        nav_data.nav_state = bt::NavState::RUNNING;
    } else if (msg->data == 2) {
        nav_data.nav_state = bt::NavState::FAILURE;
    } else if (msg->data == 3) {
        nav_data.nav_state = bt::NavState::SUCCEEDED;
    }
};
void TestNode::fold_callback(const std_msgs::msg::Bool::SharedPtr msg) {
    if (msg->data == true) {
        spdlog::info("[test]fold:");
    }
}
void TestNode::rm_bt_callback() {
    rm_decision_.tree_tick(game_data, nav_data, std::chrono::milliseconds(1));
    auto [game, nav] = rm_decision_.get_data();
    if (nav.has_value()) {
        geometry_msgs::msg::PoseStamped goal;
        goal.pose.position.x = nav->goal_point.x;
        goal.pose.position.y = nav->goal_point.y;
        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, nav->goal_point.yaw);
        q.normalize(); // 保险起见归一化

        goal.pose.orientation = tf2::toMsg(q);
        spdlog::info("[test]pub goal:({},{},{})", goal.pose.position.x, goal.pose.position.y, nav->goal_point.yaw);
        goal_pub_->publish(goal);
    }
}
void TestNode::init_game() {
    game_data.projectile_allowance = 600;
    game_data.current_hp = 500;
    game_data.current_enemy_outpost_hp = 1000;
    nav_data.nav_state = bt::NavState::IDLE;
}
void TestNode::rm_game_callback() {
    // 计算距离节点启动过了多少秒
    auto now = std::chrono::steady_clock::now();
    double elapsed = std::chrono::duration<double>(now - start_time_).count();

    if (elapsed > 420) {
        spdlog::info("game is 结束");
        return;
    }
    // ---- 5s 后 is_game_start = true ----
    if (elapsed >= 1.0) {
        game_data.is_game_start = true;
        //std::cout << "[t=" << elapsed << "s] is_game_start = true\n";
    }

    // ---- 10s 后 current_hp = 10 ----
    if (elapsed >= 10.0) {
        game_data.current_hp = 10;
        //std::cout << "[t=" << elapsed << "s] current_hp = 10\n";
    }

    // 其他字段也可以按需要在这里赋值……
    game_data.game_time = static_cast<int>(elapsed);
}
}