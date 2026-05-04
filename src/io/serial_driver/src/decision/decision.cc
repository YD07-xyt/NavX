#include "decision.h"
#include "type.h"
#include <iostream>
#include <rclcpp/logging.hpp>
namespace decision {
FSMRos2::FSMRos2(rclcpp::Node::SharedPtr node)
    : node_(node), last_sent_goal_(0, 0, 0), patrol_(node_),
      waitStartTime(std::chrono::steady_clock::now()),
      nav_start_time_(std::chrono::steady_clock::now()), // 添加
      nav_end_time_(std::chrono::steady_clock::now()),
      wait_start_time(std::chrono::steady_clock::now()),
      nav_start_time(std::chrono::steady_clock::now()), // 添加
      nav_end_time(std::chrono::steady_clock::now())  {
        
  this->goal_pub_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>(
      "/goal_pose", 10);
  this->nav2_status_sub_ =
      node_->create_subscription<action_msgs::msg::GoalStatusArray>(
          "/navigate_to_pose/_action/status", 10,
          std::bind(&FSMRos2::nav2_status_callback, this,
                    std::placeholders::_1));
}
void FSMRos2::decision(int is_game, int current_hp, int projectile_allowance,
                       int is_enemy_outpost_destroyed, int game_time) {
  if (!is_game) {
    RCLCPP_INFO(node_->get_logger(), "game is not start");
    return;
  }
  //RCLCPP_INFO(node_->get_logger(),"is_enemy_outpost_destroyed:%d",is_enemy_outpost_destroyed);
  state_enemy_outpost(is_enemy_outpost_destroyed,game_time);
  is_good_robot_condition(current_hp, projectile_allowance);
  SwitchpatrolState();

  ExecuteGameTask();
};

void FSMRos2::state_enemy_outpost(int is_enemy_outpost_destroyed,int game_time) {
  //1 -->not_destroyed  0--->  destroyed
  if (is_enemy_outpost_destroyed == 1) {
    RCLCPP_INFO(node_->get_logger(),"to HitEnemyOutpost");
    this->enemy_outpost_state_ = EnemyOutpostState::not_destroyed;
    if(game_time>=240&&game_time<=420){
      if(current_game_task_==GameTask::Free){
        current_game_task_ = GameTask::HitEnemyOutpost;
      }
    }else{
      RCLCPP_INFO(node_->get_logger(),"game_time>180||game_time<1");
    }
  } else {
    this->enemy_outpost_state_ = EnemyOutpostState::destroyed;
    current_game_task_ = GameTask::Free;
  }
}

void FSMRos2::is_good_robot_condition(int current_hp,
                                      int projectile_allowance) {
  if (current_hp >= state_is_go_home_.become_home_hp && 
    projectile_allowance > state_is_go_home_.become_home_projectile_allowance) {
    robot_state_ = RobotState::OkBecomeHome;
    RCLCPP_INFO(node_->get_logger(),"OkBecomeHome now projectile_allowance,%d",projectile_allowance);
    return;
  }
  if (current_hp < state_is_go_home_.go_home_hp || 
      projectile_allowance < state_is_go_home_.go_home_projectile_allowance) {
    robot_state_ = RobotState::need2home;
    current_game_task_ = GameTask::Gohome;
    RCLCPP_INFO(node_->get_logger(),"need to home");
    return;
  }
  robot_state_ = RobotState::Normal;
}

void FSMRos2::SwitchpatrolState() {
  if (current_game_task_ == GameTask::Free) {
    if (current_game_task_ != GameTask::Gohome &&
        current_game_task_ != GameTask::HitEnemyOutpost) {
      current_game_task_ = GameTask::PatrolA;
    }
  }
}

void FSMRos2::hit_enemy_outpost() {
  if(!is_temp_hit_point){
    Point temp_goal_point(5.01,-0.255,0);
    RCLCPP_INFO(node_->get_logger(),"is_temp_hit_point = false");
    if(nav2_state_!=Nav2State::running){
      pub_goal(temp_goal_point);
    }
  }
  printf_nav2_state();
  if(nav2_state_ == Nav2State::succeeded){
    is_temp_hit_point=true;
    pub_goal(goal_point_sum_.HitOutpost);
    RCLCPP_INFO(node_->get_logger(),"send HitOutpost");
    if (nav2_state_ == Nav2State::succeeded) {
      // 处理击毁敌方哨站后的逻辑
      if (enemy_outpost_state_ == EnemyOutpostState::destroyed) {
        RCLCPP_INFO(node_->get_logger(), "敌方哨站已被击毁，执行后续任务");
        // 可以在这里切换到其他任务或状态
        current_game_task_ = GameTask::Free; // 切换回空闲
        nav2_state_ = Nav2State::idle;
        printf_nav2_state();
      }
    }
  }
}
void FSMRos2::printf_nav2_state(){
  switch(nav2_state_){
    case Nav2State::idle:
      RCLCPP_INFO(node_->get_logger(), "🚀 导航idle");
      break;
    case Nav2State::aborted:
      RCLCPP_INFO(node_->get_logger(), "🚀 导航aborted");
      break;
    case Nav2State::succeeded:
      RCLCPP_INFO(node_->get_logger(), "🚀 导航succeeded");
      break;
    case Nav2State::running:
      RCLCPP_INFO(node_->get_logger(), "🚀 导航running");
      break;
  }
}
void FSMRos2::go_home() {
  // if (nav2_state_ == Nav2State::succeeded) {
  //   RCLCPP_INFO(node_->get_logger(), "已回到基地，执行后续任务");
  //   if (this->robot_state_ == RobotState::OkBecomeHome) {
  //     current_game_task_ = GameTask::Free; // 切换回空闲状态
  //     return;
  //   }
  // }
  if (this->robot_state_ == RobotState::OkBecomeHome) {
      current_game_task_ = GameTask::Free; // 切换回空闲状态
      return;
  }
  pub_goal(goal_point_sum_.home);
}

void FSMRos2::ExecuteGameTask() {
  if (current_game_task_ == GameTask::Gohome) {
    go_home();
    RCLCPP_INFO(node_->get_logger(), "执行回基地任务");
    return;
  }
  // if (current_game_task_ != GameTask::Free) {
  //   RCLCPP_INFO(node_->get_logger(), "ExecuteGameTask时 当前有任务在执行");
  //   return;
  // }
   // 🔧 修复：如果任务完成且 nav2_state_ 卡在 succeeded，强制重置
  switch (current_game_task_) {
  case GameTask::HitEnemyOutpost:
    hit_enemy_outpost();
    RCLCPP_INFO(node_->get_logger(), "执行击毁敌方哨站任务");
    break;
  case GameTask::PatrolA:
    patrolA();
   //RCLCPP_INFO(node_->get_logger(), "执行巡逻任务A");
    break;
  case GameTask::PatrolB:
    patrolB();
    RCLCPP_INFO(node_->get_logger(), "执行巡逻任务B");
    break;
  case GameTask::Free:
    RCLCPP_INFO(node_->get_logger(), "free");
    break;
  default:
    RCLCPP_INFO(node_->get_logger(), "未知任务");
    break;
  }
}
void FSMRos2::patrolA() {
  static bool waiting_for_nav = false;  // 添加静态变量跟踪状态
  
  // 只在未发送导航目标时获取并发送
  if (!goal_sent_) {
    auto goal = this->patrol_.selectTarget();
    printf_nav2_state();
    // if (nav2_state_ == Nav2State::idle || nav2_state_ == Nav2State::aborted) {
      pub_goal(goal);
      current_goal_point_ = goal;
      goal_sent_ = true;
      waiting_for_nav = false;
      nav_start_time = std::chrono::steady_clock::now();
      RCLCPP_INFO(node_->get_logger(), "发送新导航目标: (%.2f, %.2f)", goal.x, goal.y);
    //}
  }

  switch (nav2_state_) {
  case Nav2State::succeeded:
    if (goal_sent_) {
      // 计算导航时间
      nav_end_time = std::chrono::steady_clock::now();
      nav_time = std::chrono::duration_cast<std::chrono::milliseconds>(
                        nav_end_time - nav_start_time).count();
      
      // 如果还没有进入等待状态，开始计时等待
      if (!waiting_for_nav) {
        wait_start_time = std::chrono::steady_clock::now();
        waiting_for_nav = true;
        RCLCPP_INFO(node_->get_logger(), "到达目标点，开始等待");
      }
      
      // 计算等待时间
      wait_time = std::chrono::duration_cast<std::chrono::milliseconds>(
                        std::chrono::steady_clock::now() - wait_start_time).count()/1000.0;
      
      // 检查等待时间是否满足要求
      double required_wait_time = 0;
      switch (current_patrol_index_) {
        case 0: required_wait_time = patrol_.patrol_wait_time_.wait_point1_time; break;
        case 1: required_wait_time = patrol_.patrol_wait_time_.wait_point2_time; break;
        case 2: required_wait_time = patrol_.patrol_wait_time_.wait_point3_time; break;
        case 3: required_wait_time = patrol_.patrol_wait_time_.wait_point4_time; break;
      }
      
      if (wait_time >= required_wait_time) {
        // 等待时间足够，切换到下一个点
        patrol_.advancePatrolIndex(wait_time);
        goal_sent_ = false;
        waiting_for_nav = false;
        nav2_state_ = Nav2State::idle;
        RCLCPP_INFO(node_->get_logger(), "等待完成(%.2f ms)，切换到下一个巡逻点", wait_time);
      }
    }
    break;
    
  case Nav2State::aborted:
    if (goal_sent_) {
      goal_sent_ = false;
      waiting_for_nav = false;
      RCLCPP_WARN(node_->get_logger(), "导航失败，将重新尝试");
    }
    break;
    
  case Nav2State::running:
    // 导航中，可以输出调试信息（降低频率）
    static int counter = 0;
    if (++counter % 50 == 0) {
      //RCLCPP_INFO(node_->get_logger(), "导航中...");
    }
    break;
    
  default:
    break;
  }
}
// void FSMRos2::patrolA() {
//   //实现巡逻任务A的逻辑
//   //RCLCPP_INFO(node_->get_logger(), "正在执行巡逻任务A");
//   // 可以在这里调用patrol_对象的方法来执行具体的巡逻行为


//   wait_start_time=std::chrono::steady_clock::now();
//   RCLCPP_INFO(node_->get_logger(),"wait_time: %f",wait_time);
//   RCLCPP_INFO(node_->get_logger(),"nav_time: %f",nav_time);
//   auto goal = this->patrol_.selectTarget();
//   if(nav2_state_!=Nav2State::running){
//     pub_goal(goal);
//     nav_start_time = std::chrono::steady_clock::now();
//   }
//   switch (nav2_state_) {
//   case Nav2State::aborted:
//     // 导航失败，保持当前巡逻点不变，等待重新导航
//     RCLCPP_INFO(node_->get_logger(), "导航失败，等待重新导航");
//     break;
//   case Nav2State::succeeded:
//     // 导航成功，切换到下一个巡逻点
//     RCLCPP_INFO(node_->get_logger(), "导航成功advancePatrolIndex");
//     nav_end_time=std::chrono::steady_clock::now();
//     nav_time=std::chrono::duration_cast<std::chrono::milliseconds>(
//                       nav_end_time -nav_start_time)
//                       .count();
//     wait_time=std::chrono::duration_cast<std::chrono::milliseconds>(
//                       std::chrono::steady_clock::now() -wait_start_time)
//                       .count();
//     wait_time-=nav_time;
//     this->patrol_.advancePatrolIndex(wait_time);
//     break;
//   case Nav2State::running:
//     // 导航中，不做任何改变
//      RCLCPP_INFO(node_->get_logger(),"导航中，不做任何改变");
//     break;
//   default:
//     break;
//   }
// }
void FSMRos2::patrolB() {
  // 实现巡逻任务B的逻辑
  RCLCPP_INFO(node_->get_logger(), "正在执行巡逻任务B");
  std::chrono::steady_clock::time_point nav_end_time;
  std::chrono::steady_clock::time_point nav_start_time;
  std::chrono::steady_clock::time_point wait_start_time;
  double wait_time;
  double nav_time;
  wait_start_time=std::chrono::steady_clock::now();
  // 可以在这里调用patrol_对象的方法来执行具体的巡逻行为
  auto goal = this->patrol_.selectTarget();
  if(nav2_state_!=Nav2State::running){
    pub_goal(goal);
    nav_start_time= std::chrono::steady_clock::now();
  }
  switch (nav2_state_) {
  case Nav2State::aborted:
    // 导航失败，保持当前巡逻点不变，等待重新导航
    RCLCPP_INFO(node_->get_logger(), "导航失败，等待重新导航");
    break;
  case Nav2State::succeeded:
    // 导航成功，切换到下一个巡逻点
    RCLCPP_INFO(node_->get_logger(), " 导航成功advancePatrolIndex");
    nav_end_time=std::chrono::steady_clock::now();
    nav_time=std::chrono::duration_cast<std::chrono::milliseconds>(
                      nav_end_time -nav_start_time)
                      .count();
    wait_time=std::chrono::duration_cast<std::chrono::milliseconds>(
                      std::chrono::steady_clock::now() -wait_start_time)
                      .count();
    wait_time-=nav_time;
    this->patrol_.advancePatrolIndex(wait_time);
    break;
  case Nav2State::running:
    // 导航中，不做任何改变
    break;
  default:
    break;
  }
}



///==================================================================================///
///==================================================================================///
///==================================================================================///
void FSMRos2::decision(int is_game, int current_hp, int projectile_allowance) {
  if (!is_game) {
    RCLCPP_INFO(node_->get_logger(), "game is not start");
    return;
  }
  decision::Point target_goal(0, 0, 0);
  if (this->nav2_status_ == 2) {
    //导航执行中
    // RCLCPP_INFO(node_->get_logger(), "🚀 导航执行中, not pub goal");
    if (current_hp > 200 && projectile_allowance > 0) {
      // RCLCPP_INFO(node_->get_logger(), "🚀 导航执行中, not pub goal");
      return;
    }
    // RCLCPP_INFO(node_->get_logger(), "🚀 导航执行中----->home!");
  }

  if (this->nav2_status_ == 4) {
    RCLCPP_INFO(node_->get_logger(), "✅ 导航成功！");
    this->nav_end_time_ = std::chrono::steady_clock::now();
    advancePatrolIndex();
    nav2_status_ = 0; // 重置状态，避免重复切换
  }

  target_goal = this->selectTarget(current_hp, projectile_allowance);

  pub_goal(target_goal);
  this->nav_start_time_ = std::chrono::steady_clock::now();
  this->last_sent_goal_ = target_goal;
}

// 分离：选择目标点
Point FSMRos2::selectTarget(int current_hp, int projectile_allowance) {
  // 紧急情况
  if (current_hp < 200 || projectile_allowance < 0) {
    current_patrol_index_ = 0; // 重置巡逻
    return goal_point_sum_.home;
  }
  if (current_hp >= 400 && projectile_allowance > 1) {
    // 正常巡逻
    switch (current_patrol_index_) {
    case 0:
      return goal_point_sum_.Patrol1;
    case 1:
      return goal_point_sum_.Patrol2;
    // case 2: return goal_point_sum_.Patrol3;
    default:
      current_patrol_index_ = 0;
      return goal_point_sum_.Patrol1;
    }
  }
  return Point(0, 0, 0);
}
void FSMRos2::advancePatrolIndex() {
  // 导航耗时
  auto nav_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                    nav_end_time_ - nav_start_time_)
                    .count();
  double nav_time = nav_ms / 1000.0;

  // 总耗时（从上次发目标开始算）
  auto total_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                      std::chrono::steady_clock::now() - waitStartTime)
                      .count();

  // 纯等待时间
  long long elapsed_ms = total_ms - nav_ms;
  if (elapsed_ms < 0)
    elapsed_ms = 0;
  double elapsed = elapsed_ms / 1000.0;

  RCLCPP_INFO(node_->get_logger(), "nav_time: %.6f ms", nav_time);
  RCLCPP_INFO(node_->get_logger(), "elapsed time:%.0f ms", elapsed);

  bool need_go = false;

  switch (current_patrol_index_) {
  case 0:
    if (elapsed >= wait_point1_time_) {

      current_patrol_index_ = 1;
      need_go = true;
      RCLCPP_INFO(node_->get_logger(), "Patrol1 -> Patrol2");
    }
    break;

  case 1:
    if (elapsed >= wait_point2_time_) {
      current_patrol_index_ = 0;
      need_go = true;
      RCLCPP_INFO(node_->get_logger(), "Patrol2 -> Patrol1 (loop)");
      break;
    }

    if (need_go) {
      waitStartTime = std::chrono::steady_clock::now();
      nav2_status_ = 0; // 切换为空闲，允许发新目标
    }
  }
}

void FSMRos2::pub_goal(Point goal_point) {
  auto msg = geometry_msgs::msg::PoseStamped();

  // 设置时间戳和坐标系
  msg.header.stamp = node_->now();
  msg.header.frame_id = this->fsm_config_.map_tf_name_; //!!!

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
  RCLCPP_INFO(node_->get_logger(), "Published goal: (%.2f, %.2f, %.2f rad)",
              goal_point.x, goal_point.y, goal_point.yaw);
}
void FSMRos2::nav2_status_callback(const action_msgs::msg::GoalStatusArray& msg) {
  if (msg.status_list.empty()) return;
  
  // 获取最新的状态
  auto& last_status = msg.status_list.back();
  
  if (last_status.status == action_msgs::msg::GoalStatus::STATUS_SUCCEEDED) {
    if (nav2_state_ != Nav2State::succeeded) {
      nav2_state_ = Nav2State::succeeded;
      RCLCPP_INFO(node_->get_logger(), "✅ 导航成功！");
    }
  } else if (last_status.status == action_msgs::msg::GoalStatus::STATUS_ABORTED) {
    if (nav2_state_ != Nav2State::aborted) {
      nav2_state_ = Nav2State::aborted;
      RCLCPP_ERROR(node_->get_logger(), "❌ 导航失败/终止");
    }
  } else if (last_status.status == action_msgs::msg::GoalStatus::STATUS_EXECUTING) {
    if (nav2_state_ != Nav2State::running) {
      nav2_state_ = Nav2State::running;
      RCLCPP_INFO(node_->get_logger(), "🚀 导航执行中");
    }
  }
}
// void FSMRos2::nav2_status_callback(
//     const action_msgs::msg::GoalStatusArray msg) {
//   for (const auto &status : msg.status_list) {
//     if (status.status == action_msgs::msg::GoalStatus::STATUS_SUCCEEDED) {
//       this->nav2_status_ = 4;
//       nav2_state_ = Nav2State::succeeded;
//       this->nav_end_time_ = std::chrono::steady_clock::now();
//       RCLCPP_INFO(node_->get_logger(), "✅ 导航成功！");
//     } else if (status.status == action_msgs::msg::GoalStatus::STATUS_ABORTED) {

//       this->nav2_status_ = 6;
//       nav2_state_ = Nav2State::aborted;
//       // RCLCPP_ERROR(node_->get_logger(), "❌ 导航失败/终止");
//     } else if (status.status == action_msgs::msg::GoalStatus::STATUS_CANCELED) {
//       this->nav2_status_ = 5;
//       nav2_state_ = Nav2State::aborted;
//       // RCLCPP_WARN(node_->get_logger(), "⚠️ 导航被取消");
//     } else if (status.status ==
//                action_msgs::msg::GoalStatus::STATUS_EXECUTING) {

//       this->nav2_status_ = 2;
//       nav2_state_ = Nav2State::running;
//       // RCLCPP_INFO(node_->get_logger(), "🚀 导航执行中");
//     }
//   }
//   // 打印更多信息
//   // RCLCPP_INFO(node_->get_logger(),
//   //             "Goal ID: %s, Status: %d",
//   //             msg->goal_id.c_str(),
//   //             msg->status);
// }
} // namespace decision