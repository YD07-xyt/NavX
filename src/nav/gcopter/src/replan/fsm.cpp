#include "utils/logger.hpp"
#include <algorithm>
#include <chrono>
#include <fsm/fsm.hpp>
#include <limits>
namespace planner {

// steady_clock 秒（供路径年龄 / 防抖计时用）
static double now_sec() {
    return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}

auto FSM::set_astar_param(path_planning::AStar& astar) {
    astar.setMaxVelocity(fsm_config_.astar_param_.max_vel_);
    astar.setMaxAcceleration(fsm_config_.astar_param_.max_acc_);
    astar.setTimeResolution(fsm_config_.astar_param_.time_resolution_);
    astar.setMinTrajectoryNumber(fsm_config_.astar_param_.min_traj_num_);
    astar.setTrajectoryCutLength(fsm_config_.astar_param_.traj_cut_length_);
    astar.setDistanceWeight(fsm_config_.astar_param_.distance_weight_);
    astar.setYawWeight(fsm_config_.astar_param_.yaw_weight_);
}

auto FSM::plan(
    const utils::RobotState& goal_pose,
    const utils::RobotState& current_pose,
    std::shared_ptr<grid_map::GridMap> grid_map
) -> astar_opt_path {
    if (checkPointEqual(old_goal_pose_.p, goal_pose.p, fsm_config_.deviation)) {
        // 目标没变，但需要检查当前已规划路径是否仍然有效
        if (path_state_ == PathState::running || path_state_ == PathState::successed) {
            // 触发重规划的三个条件（满足任一即重规划）：
            //   1. 路径碰障（A* 原始网格路径 + 优化后航点路径，见 checkCollision）
            //   2. 路径年龄超过 replan_interval_
            //   3. 机器人偏离参考路径超过 replan_lateral_dev_
            const double now = now_sec();
            bool need_replan = false;
            if (astar_traj_.optimized_path.empty() || checkCollision(astar_traj_)) {
                need_replan = true;
            } else if (now - last_plan_time_ > fsm_config_.replan_interval_) {
                need_replan = true;
            } else if (lateralDeviation(Eigen::Vector2d(current_pose.p.x(), current_pose.p.y()),
                                        last_opt_path_.empty() ? astar_traj_.optimized_path
                                                               : last_opt_path_) > fsm_config_.replan_lateral_dev_) {
                need_replan = true;
            }
            if (!need_replan) {
                // 路径仍然有效，且目标未变，可以直接返回成功（避免重复规划）
                return tl::make_unexpected(path_error::success);
            }
            // 防抖：距上次重规划不足 min_replan_interval_ 时保留旧路径，下一拍再试
            if (now - last_replan_time_ < fsm_config_.min_replan_interval_) {
                return tl::make_unexpected(path_error::success);
            }
            logger::fsm_replan->debug("Path invalid (collision/stale/deviation), triggering replanning...");
            path_state_ = PathState::idle;
            // 继续执行后续的重规划逻辑（不返回）
        } else {
            // 其他状态（如 idle, failed）也应该继续尝试规划
        }
    }
    if (!checkPointEqual(old_goal_pose_.p, goal_pose.p, Eigen::Vector3d::Zero())) {
        logger::fsm_replan->debug("plan goal is change");
        old_goal_pose_ = goal_pose;
        path_state_ = PathState::idle;
    }

    // 起点≈终点（机器人已到达目标附近）：直接退出，不规划。
    // 否则 A* 返回单点退化路径、样条优化无意义，且周期重规划会反复触发失败日志
    if ((current_pose.p.head<2>() - goal_pose.p.head<2>()).norm() < fsm_config_.goal_reached_radius_) {
        return tl::make_unexpected(path_error::success);
    }

    // failed 状态也限频重试：A*/优化失败时不要每 33ms 空转刷屏
    if (path_state_ == PathState::failed && now_sec() - last_replan_time_ < fsm_config_.min_replan_interval_) {
        return tl::make_unexpected(path_error::success);
    }

    // 2. 准备数据
    grid_map_ = grid_map;

    Eigen::Vector2d start(current_pose.p.x(), current_pose.p.y());
    Eigen::Vector2d goal(goal_pose.p.x(), goal_pose.p.y());

    // 若起点/终点过于靠近障碍物，沿 ESDF 梯度外推到安全点，保证可规划
    start = getSafeStart(start);
    goal = getSafeStart(goal);
    Eigen::Vector2d start_vel(current_pose.v.x(), current_pose.v.y());
    Eigen::Vector2d goal_vel(0.0, 0.0); // 默认到目标停车
    // 3. 带重试的规划循环
    const int max_retries = 100;
    int retry_count = 0;
    path_planning::AStar astar(*grid_map, fsm_config_.safe_threshold_);
    set_astar_param(astar);
    astar.setStartVelocity(start_vel.norm());
    astar.setEndVelocity(goal_vel.norm());
    while (path_state_ != PathState::running && retry_count < max_retries) {
        auto astar_traj = astar.planWithPostProcessing(start, goal, 5000);
        astar_traj_ = astar_traj;
        old_goal_pose_ = goal_pose;

        if (astar_traj.optimized_path.empty()) {
            logger::fsm_replan->error("A* planning failed!");
            logger::fsm_replan->error(
                "  [diag] safe_threshold={:.3f} map_size=({:.2f},{:.2f}) "
                "resolution={:.4f}",
                fsm_config_.safe_threshold_,
                grid_map_->getMapSize().x(),
                grid_map_->getMapSize().y(),
                grid_map_->getResolution()
            );
            logger::fsm_replan->error(
                "  [diag] start=({:.3f},{:.3f}) goal=({:.3f},{:.3f}) "
                "start_dist={:.3f} goal_dist={:.3f}",
                start.x(),
                start.y(),
                goal.x(),
                goal.y(),
                grid_map_->getDistance(start),
                grid_map_->getDistance(goal)
            );
            path_state_ = PathState::failed;
            last_replan_time_ = now_sec(); // 失败也吃防抖，避免 30Hz 空转
            return tl::make_unexpected(path_error::astar_path_empty);
        }

        // 碰撞检测
        if (checkCollision(astar_traj_)) {
            logger::fsm_replan->warn("Collision detected, retrying... (attempt {}/{})", retry_count + 1, max_retries);
            path_state_ = PathState::idle; // 重置为 idle 以继续循环
            retry_count++;
            continue; // 重新执行 A*
        } else {
            // logger::fsm_replan->info("collision is not failed");
        }

        // 无碰撞，进入优化阶段
        path_state_ = PathState::running;

        auto start_time = std::chrono::high_resolution_clock::now();
        // 防止 total_time 为 0 时产生 NaN（短/退化路径的兜底，正常情况下不会被触发）
        const double safe_total_time = (astar_traj.total_time > 1e-6) ? astar_traj.total_time : 1e-6;
        fsm_config_.params_.piece_len = astar_traj.total_length / safe_total_time;
        fsm_config_.params_.total_time = astar_traj.total_time;
        fsm_config_.params_.total_len = astar_traj.total_length;

        TrajOpt::TrajectoryOptimizer optimizer(grid_map, astar_traj.optimized_path, fsm_config_.params_);
        optimizer.set_start_vel(start_vel);
        optimizer.set_end_vel(goal_vel);
        if (!optimizer.plan()) {
            logger::fsm_replan->warn(
                "[opt-fail] astar: {} pts, len={:.3f} m, t={:.3f} s, start=({:.2f},{:.2f}), goal=({:.2f},{:.2f})",
                astar_traj.optimized_path.size(),
                astar_traj.total_length,
                astar_traj.total_time,
                start.x(), start.y(), goal.x(), goal.y()
            );
            path_state_ = PathState::failed;
            last_replan_time_ = now_sec(); // 失败也吃防抖，避免 30Hz 空转
            return tl::make_unexpected(path_error::optimizer_failed);
        }

        auto end_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
        auto metrics = optimizer.evaluateTrajectory();
        logger::fsm_replan->debug(
            "[speed-diag] astar_total_time={:.3f}s astar_total_len={:.3f}m "
            "traj_duration={:.3f}s traj_max_vel={:.3f}m/s",
            astar_traj.total_time,
            astar_traj.total_length,
            optimizer.getOptimizedTrajectory().getDuration(),
            metrics.max_velocity
        );

        std::vector<Eigen::Vector2d> opt_path = optimizer.sampleTrajectory(0.1);
        // 记录成功规划时间（供路径年龄 / 防抖使用）与优化轨迹采样点（供横向偏差检测使用）
        last_plan_time_ = now_sec();
        last_replan_time_ = last_plan_time_;
        last_opt_path_ = optimizer.sampleTrajectory(0.2);
        return std::make_pair(astar_traj.optimized_path, optimizer);
    }

    // 循环结束仍未成功（状态不是 running 或重试耗尽）
    if (retry_count >= max_retries) {
        logger::fsm_replan->error("Max retries reached, planning failed due to collisions");
        path_state_ = PathState::failed;
        return tl::make_unexpected(path_error::astar_path_empty);
    }

    return tl::make_unexpected(path_error::none);
}

auto FSM::getSafeStart(const Eigen::Vector2d& pos) -> Eigen::Vector2d {
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
        if (grid_map_->getDistance(safe) >= fsm_config_.safe_threshold_ || pushed >= max_push) {
            return safe;
        }
    }
    return safe;
}

auto FSM::checkCollision(path_planning::AStar::Trajectory astar_traj) -> bool {
    // 同时检查 A* 原始网格路径与优化后航点路径：
    // 原始路径逐格密集，可发现两个优化航点之间被新障碍挡住的情况
    std::vector<std::vector<Eigen::Vector2d>> paths;
    paths.push_back(astar_traj.optimized_path);
    paths.push_back(astar_traj.raw_path);
    for (const auto& path : paths) {
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
    }
    return false;
};

auto FSM::lateralDeviation(const Eigen::Vector2d& pos, const std::vector<Eigen::Vector2d>& path) -> double {
    if (path.size() < 2) {
        return std::numeric_limits<double>::max();
    }
    // 点到折线各段的最短距离（逐段投影并夹紧到线段内）
    double min_dist = std::numeric_limits<double>::max();
    for (size_t i = 1; i < path.size(); ++i) {
        const Eigen::Vector2d a = path[i - 1];
        const Eigen::Vector2d b = path[i];
        const Eigen::Vector2d ab = b - a;
        const double len2 = ab.squaredNorm();
        double t = 0.0;
        if (len2 > 1e-12) {
            t = std::max(0.0, std::min(1.0, (pos - a).dot(ab) / len2));
        }
        const Eigen::Vector2d closest = a + t * ab;
        min_dist = std::min(min_dist, (pos - closest).norm());
    }
    return min_dist;
}

auto FSM::checkPointEqual(const Eigen::Vector3d& pos1, const Eigen::Vector3d& pos2, const Eigen::Vector3d& deviation)
    -> bool {
    if (deviation == Eigen::Vector3d::Zero()) {
        if (std::abs(pos1.x() - pos2.x()) < std::numeric_limits<double>::epsilon()
            && std::abs(pos1.y() - pos2.y()) < std::numeric_limits<double>::epsilon())
        {
            return true;
        }
        return false;
    }
    if (std::abs(pos1.x() - pos2.x()) < deviation.x() && std::abs(pos1.y() - pos2.y()) < deviation.y()) {
        return true;
    }
    return false;
}
} // namespace planner