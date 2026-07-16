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
  auto set_astar_param(path_planning::AStar &astar) {
    astar.setMaxVelocity(fsm_config_.astar_param_.max_vel_);
    astar.setMaxAcceleration(fsm_config_.astar_param_.max_acc_);
    astar.setTimeResolution(fsm_config_.astar_param_.time_resolution_);
    astar.setMinTrajectoryNumber(fsm_config_.astar_param_.min_traj_num_);
    astar.setTrajectoryCutLength(fsm_config_.astar_param_.traj_cut_length_);
    astar.setDistanceWeight(fsm_config_.astar_param_.distance_weight_);
    astar.setYawWeight(fsm_config_.astar_param_.yaw_weight_);
  }
  auto plan(const Eigen::Vector3d &goal_pose,
            const Eigen::Vector3d &current_pose,
            std::shared_ptr<grid_map::GridMap> grid_map) -> astar_opt_path {

    if (checkPointEqual(old_goal_pose_, goal_pose, fsm_config_.deviation)) {
      // 目标没变，但需要检查当前已规划路径是否仍然安全
      if (path_state_ == PathState::running ||
          path_state_ == PathState::successed) {
        // 重新检查上次的 astar_traj_ 是否发生碰撞
        if (!astar_traj_.optimized_path.empty() &&
            !checkCollision(astar_traj_)) {
          // 路径仍然安全，且目标未变，可以直接返回成功（避免重复规划）
          return tl::make_unexpected(path_error::success);
        } else {
          // 路径不再安全，重置状态，准备重规划
          spdlog::warn("Path became unsafe, triggering replanning...");
          path_state_ = PathState::idle;
          // 继续执行后续的重规划逻辑（不返回）
        }
      } else {
        // 其他状态（如 idle, failed）也应该继续尝试规划
      }
    }
    if (!checkPointEqual(old_goal_pose_, goal_pose, Eigen::Vector3d::Zero())) {
      spdlog::info("plan goal is change");
      old_goal_pose_ = goal_pose;
      path_state_ = PathState::idle;
    }

    // 2. 准备数据
    grid_map_ = grid_map;

    Eigen::Vector2d start(current_pose.x(), current_pose.y());
    Eigen::Vector2d goal(goal_pose.x(), goal_pose.y());

    // 若起点/终点过于靠近障碍物，沿 ESDF 梯度外推到安全点，保证可规划
    start = getSafeStart(start);
    goal = getSafeStart(goal);

    // 3. 带重试的规划循环
    const int max_retries = 30;
    int retry_count = 0;
    path_planning::AStar astar(*grid_map, fsm_config_.safe_threshold_);
    set_astar_param(astar);
    while (path_state_ != PathState::running && retry_count < max_retries) {

      auto astar_traj = astar.planWithPostProcessing(start, goal, 5000);
      astar_traj_ = astar_traj;
      old_goal_pose_ = goal_pose;

      // spdlog::info("old_goal_pose_: ({:.2f}, {:.2f})", old_goal_pose_.x(),
      //              old_goal_pose_.y());

      if (astar_traj.optimized_path.empty()) {
        spdlog::error("A* planning failed!");
        spdlog::error(
            "  [diag] safe_threshold={:.3f} map_size=({:.2f},{:.2f}) "
            "resolution={:.4f}",
            fsm_config_.safe_threshold_, grid_map_->getMapSize().x(),
            grid_map_->getMapSize().y(), grid_map_->getResolution());
        spdlog::error(
            "  [diag] start=({:.3f},{:.3f}) goal=({:.3f},{:.3f}) "
            "start_dist={:.3f} goal_dist={:.3f}",
            start.x(), start.y(), goal.x(), goal.y(),
            grid_map_->getDistance(start), grid_map_->getDistance(goal));
        path_state_ = PathState::failed;
        return tl::make_unexpected(path_error::astar_path_empty);
      }

      // 碰撞检测
      if (checkCollision(astar_traj_)) {
        spdlog::warn("Collision detected, retrying... (attempt {}/{})",
                     retry_count + 1, max_retries);
        path_state_ = PathState::idle; // 重置为 idle 以继续循环
        retry_count++;
        continue; // 重新执行 A*
      } else {
        // spdlog::info("collision is not failed");
      }

      // 无碰撞，进入优化阶段
      path_state_ = PathState::running;

      auto start_time = std::chrono::high_resolution_clock::now();
      fsm_config_.params_.piece_len =
          astar_traj.total_length / astar_traj.total_time;
      fsm_config_.params_.total_time = astar_traj.total_time;
      fsm_config_.params_.total_len = astar_traj.total_length;

      TrajOpt::TrajectoryOptimizer optimizer(
          grid_map, astar_traj.optimized_path, fsm_config_.params_);
      if (!optimizer.plan()) {
        spdlog::error("Trajectory optimization failed!");
        path_state_ = PathState::failed;
        return tl::make_unexpected(path_error::optimizer_failed);
      }

      auto end_time = std::chrono::high_resolution_clock::now();
      auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(
          end_time - start_time);
      auto metrics = optimizer.evaluateTrajectory();
      spdlog::info(
          "[speed-diag] astar_total_time={:.3f}s astar_total_len={:.3f}m "
          "traj_duration={:.3f}s traj_max_vel={:.3f}m/s",
          astar_traj.total_time, astar_traj.total_length,
          optimizer.getOptimizedTrajectory().getDuration(), metrics.max_velocity);
      std::vector<Eigen::Vector2d> opt_path = optimizer.sampleTrajectory(0.1);
      return std::make_pair(astar_traj.optimized_path, optimizer);
    }

    // 循环结束仍未成功（状态不是 running 或重试耗尽）
    if (retry_count >= max_retries) {
      spdlog::error("Max retries reached, planning failed due to collisions");
      path_state_ = PathState::failed;
      return tl::make_unexpected(path_error::astar_path_empty);
    }

    return tl::make_unexpected(path_error::none);
  }

public:
private:
  // 将过于靠近障碍物的点沿 ESDF 梯度外推到安全距离，确保 A*/碰撞检测可通过
  auto getSafeStart(const Eigen::Vector2d &pos) -> Eigen::Vector2d {
    if (!grid_map_ || !grid_map_->isInsideMap(pos)) {
      return pos;
    }
    if (grid_map_->getDistance(pos) >= fsm_config_.safe_threshold_) {
      return pos;
    }
    Eigen::Vector2d safe = pos;
    const double max_push = 1.5; // 最大外推距离，避免把起点推得太远
    double pushed = 0.0;
    for (int i = 0; i < 200; ++i) {
      double d = 0.0;
      Eigen::Vector2d g;
      grid_map_->getDistanceAndGradient(safe, d, g);
      double gn = g.norm();
      if (gn < 1e-6) {
        break; // 梯度退化，无法继续外推
      }
      Eigen::Vector2d dir = g / gn; // 梯度指向远离障碍方向
      double need = (fsm_config_.safe_threshold_ - d) + 0.05;
      double step = std::min(need, max_push - pushed);
      if (step <= 0) {
        break;
      }
      safe += dir * step;
      pushed += step;
      if (grid_map_->getDistance(safe) >= fsm_config_.safe_threshold_ ||
          pushed >= max_push) {
        return safe;
      }
    }
    return safe;
  }

  auto checkCollision(path_planning::AStar::Trajectory astar_traj) -> bool {
    auto path = astar_traj.optimized_path;
    // Points outside map are considered collision-free
    for (auto pos : path) {
      if (!grid_map_->isInsideMap(pos)) {
        continue;
      }

      // Points inside map use safety distance check
      if (grid_map_->getDistance(pos) < fsm_config_.safe_threshold_) {
        return true;
      };
    }
    return false;
  };
  auto checkPointEqual(const Eigen::Vector3d &pos1, const Eigen::Vector3d &pos2,
                       const Eigen::Vector3d &deviation) -> bool {
    if (deviation == Eigen::Vector3d::Zero()) {
      if (std::abs(pos1.x() - pos2.x()) <
              std::numeric_limits<double>::epsilon() &&
          std::abs(pos1.y() - pos2.y()) <
              std::numeric_limits<double>::epsilon()) {
        return true;
      }
      return false;
    }
    if (std::abs(pos1.x() - pos2.x()) < deviation.x() &&
        std::abs(pos1.y() - pos2.y()) < deviation.y()) {
      return true;
    }
    return false;
  }

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