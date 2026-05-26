#pragma once
#include "differential_mpc.h"
#include <Eigen/Dense>
#include <Eigen/src/Core/Matrix.h>
#include <vector>
#include <cmath>

class TrajectoryTracker {
public:
    TrajectoryTracker(double dt, int N) : dt_(dt), N_(N), mpc_() {
    }

    // 设置参考轨迹（由外部规划器生成）
    void setReferenceTrajectory(const std::vector<Eigen::Vector2d>& opt_path) {
        // 计算完整的带朝向轨迹（x,y,theta）
        full_trajectory_.clear();
        if (opt_path.size() < 2) return;

        for (size_t i = 0; i < opt_path.size(); ++i) {
            double theta = 0.0;
            if (i + 1 < opt_path.size()) {
                double dx = opt_path[i+1].x() - opt_path[i].x();
                double dy = opt_path[i+1].y() - opt_path[i].y();
                theta = std::atan2(dy, dx);
            } else {
                // 最后一个点沿用前一个点的朝向
                theta = full_trajectory_.back()[2];
            }
            full_trajectory_.push_back({opt_path[i].x(), opt_path[i].y(), theta});
        }

        // 重置窗口起始索引
        ref_start_idx_ = 0;
    }
    bool is_full_trajectory(){
        if(full_trajectory_.empty()){
            return false;
        }
        return true;
    }
    // 在每个控制周期调用：输入当前机器人状态，输出 (v, w)
    Eigen::Vector2d update (Eigen::Vector3d& current_state) {
        // 1. 滑动参考窗口：找到距离当前状态最近的点作为起始索引
        //    简单做法：如果当前已超过参考窗口的第一个点，则向前移动
        if (ref_start_idx_ + N_ + 1 <= full_trajectory_.size()) {
            // 检查是否需要滑动（例如，当前状态已经接近或超过第一个参考点）
            double dx = current_state(0) - full_trajectory_[ref_start_idx_][0];
            double dy = current_state(1) - full_trajectory_[ref_start_idx_][1];
            if (dx*dx + dy*dy < 0.1*0.1) { // 距离小于阈值则滑动
                ref_start_idx_ = std::min(ref_start_idx_ + 1,
                                          (int)full_trajectory_.size() - N_ - 1);
            }
        }

        // 2. 构建当前 MPC 需要的 desired_states 矩阵 (3 x (N+1))
        Eigen::MatrixXd desired_states(3, N_ + 1);
        for (int i = 0; i <= N_; ++i) {
            int idx = ref_start_idx_ + i;
            if (idx >= (int)full_trajectory_.size()) {
                // 如果到达轨迹末端，保持最后一个状态
                idx = full_trajectory_.size() - 1;
            }
            desired_states(0, i) = full_trajectory_[idx][0];
            desired_states(1, i) = full_trajectory_[idx][1];
            desired_states(2, i) = full_trajectory_[idx][2];
        }

        // 3. 调用 MPC 求解
        if (!mpc_.solve(current_state, desired_states)) {
            spdlog::warn("MPC solve failed, using zero control");
            return {0.0, 0.0};
        }

        // 4. 获取第一个控制量
        std::vector<double> u = mpc_.getFirstU();
        return {u[0], u[1]};
    }

private:
    double dt_;
    int N_;
    Mpc mpc_;
    std::vector<std::array<double, 3>> full_trajectory_; // x, y, theta
    int ref_start_idx_ = 0;
};