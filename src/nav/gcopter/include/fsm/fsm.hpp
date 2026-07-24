#pragma once

#include "nav.hpp"
#include <Eigen/src/Core/Matrix.h>
#include <optional>
#include <spdlog/spdlog.h>
#include <utils/expected.hpp>
namespace planner {
class FSM {
public:
  struct FSMConfig {
    TrajOpt::TrajectoryParams params_;
    float safe_threshold_ = 0.9;
    Eigen::Vector3d deviation = Eigen::Vector3d(0.5, 0.5, 0.0);
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
  auto plan(const Eigen::Vector3d &goal_pose,
            const Eigen::Vector3d &current_pose,
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
  Eigen::Vector3d old_goal_pose_;
};
} // namespace planner