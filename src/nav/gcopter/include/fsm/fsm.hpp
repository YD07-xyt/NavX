#pragma once

#include "nav.hpp"
#include "utils/type_utils.hpp"
#include <Eigen/Core>
#include <optional>
#include <spdlog/spdlog.h>
#include<utils/logger.hpp>
#include <utils/expected.hpp>
namespace planner {
class FSM {
public:
  struct FSMConfig {
    TrajOpt::TrajectoryParams params_;
    float safe_threshold_ = 0.9;
    Eigen::Vector3d deviation = Eigen::Vector3d(0.5, 0.5, 0.0);
    // ========== 重规划触发参数 ==========
    double replan_interval_ = 1.0;     // 路径最大年龄（s），超过则强制重规划
    double replan_lateral_dev_ = 0.3;  // 横向偏差阈值（m），机器人偏离参考路径超过则重规划
    double min_replan_interval_ = 0.3; // 最小重规划间隔（s），防抖，避免多触发源共振
    double goal_reached_radius_ = 0.3; // 起点≈终点判定半径（m）：机器人距目标小于该值时直接退出不规划，
                                       // 避免退化路径（A* 单点）与无意义的周期重规划
    struct AstarParam {
      double max_vel_ = 3.0;
      double max_acc_ = 1.5;
      double time_resolution_ = 0.1;
      int min_traj_num_ = 5;
      double traj_cut_length_ = 2.0;
      double distance_weight_ = 1.0;
      double yaw_weight_ = 0.5;
    } astar_param_;
  } fsm_config_;

public:
  FSM(const FSMConfig &fsm_config) : fsm_config_(fsm_config) {}
  enum path_error {
    none,
    success,
    astar_path_empty,
    optimizer_failed,
    astar_path_collision,
  };
  using path = std::vector<Eigen::Vector2d>;
  using astar_opt_path =
      tl::expected<std::pair<path, TrajOpt::TrajectoryOptimizer>, path_error>;
  auto set_astar_param(path_planning::AStar &astar);
  auto plan(const utils::RobotState &goal_pose,
            const utils::RobotState &current_pose,
            std::shared_ptr<grid_map::GridMap> grid_map) -> astar_opt_path;

public:
private:
  // 将过于靠近障碍物的点沿 ESDF 梯度外推到安全距离，确保 A*/碰撞检测可通过
  auto getSafeStart(const Eigen::Vector2d &pos) -> Eigen::Vector2d;
  /**
  @brief: 检查是否碰撞
  */
  auto checkCollision(path_planning::AStar::Trajectory astar_traj) -> bool;
  /**
  @brief: 判断2个点是否相等
  @param: 点1,2 ，允许误差
  */
  auto checkPointEqual(const Eigen::Vector3d &pos1, const Eigen::Vector3d &pos2,
                       const Eigen::Vector3d &deviation) -> bool;

  enum PathState {
    running,
    successed,
    failed,
    idle,
  } path_state_ = PathState::idle;
  // PathState opt_state_=PathState::idle;
  path_planning::AStar::Trajectory astar_traj_;
  std::shared_ptr<grid_map::GridMap> grid_map_;
  utils::RobotState old_goal_pose_;
  // 重规划触发辅助
  double last_plan_time_ = -1.0;    // 上次成功规划时间（steady_clock 秒）
  double last_replan_time_ = -1.0;  // 上次触发重规划的时间（防抖用）
  std::vector<Eigen::Vector2d> last_opt_path_; // 最近一次优化轨迹采样点（横向偏差检测用）
  /** @brief: 计算点到折线路径的横向距离（逐段最近点） */
  auto lateralDeviation(const Eigen::Vector2d &pos,
                        const std::vector<Eigen::Vector2d> &path) -> double;
};
} // namespace planner