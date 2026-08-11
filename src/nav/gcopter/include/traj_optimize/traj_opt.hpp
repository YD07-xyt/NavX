/*
    MIT License

    Copyright (c) 2025 Senming Tan (senmingtan5@gmail.com)
    Copyright (c) 2025 Deping Zhang (beiyuena@foxmail.com)

    Permission is hereby granted, free of charge, to any person obtaining a copy
    of this software and associated documentation files (the "Software"), to deal
    in the Software without restriction, including without limitation the rights
    to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
    copies of the Software, and to permit persons to whom the Software is
    furnished to do so, subject to the following conditions:

    The above copyright notice and this permission notice shall be included in all
    copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
    OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
    SOFTWARE.
*/

#ifndef TRAJ_OPT_HPP
#define TRAJ_OPT_HPP

#include "../utils/SplineTrajectory.hpp"
#include "../map/grid_map.hpp"
#include "../utils/lbfgs.hpp"
#include <spdlog/spdlog.h>
#include <vector>
#include <cmath>
#include <iostream>
#include <Eigen/Dense>

namespace TrajOpt {

struct TrajectoryParams {
    double total_len = 10; // 轨迹总长度（米）
    double total_time = 10; // 轨迹总时间（秒）
    double piece_len = 4; // 每段样条的长度（米）
    double rho_v = 100; // 速度惩罚权重（仅惩罚超速）
    double rho_v_des = 0.0; // 期望巡航速度惩罚权重（0=禁用；>0 时轨迹快速提速到巡航速度，改善起步慢）
    double v_des_ratio = 0.8; // 期望巡航速度 = v_des_ratio * max_v
    double rho_collision = 1000; // 碰撞惩罚权重
    double rho_T = 100; // 时间惩罚权重
    double rho_energy = 100; // 能量/平滑度惩罚权重
    double max_v = 3.0; // 最大线速度（米/秒）
    double safe_threshold = 0.9; // 安全距离阈值（米）
    int int_K = 32; // 每个轨迹段的采样点数
    int mem_size = 256; // L-BFGS 存储的历史梯度数量
    int past = 3; // L-BFGS 用于收敛检测的迭代次数
    double g_epsilon = 1e-6; // 梯度范数收敛阈值
    double min_step = 1e-32; // 最小步长
    double delta = 1e-5; // 函数值变化收敛阈值
    int max_iter = 10000; // 最大迭代次数
};

class TrajectoryOptimizer {
public:
    using Vector2d = Eigen::Vector2d;
    using Vector3d = Eigen::Vector3d;
    using MatrixXd = Eigen::MatrixXd;
    using VectorXd = Eigen::VectorXd;
    using PPoly2D = SplineTrajectory::PPolyND<2>;
    using CubicSpline2D = SplineTrajectory::CubicSpline2D;
    TrajectoryOptimizer() {
        in_opt_ = false;
    };
    TrajectoryOptimizer(
        std::shared_ptr<grid_map::GridMap> map,
        const std::vector<Eigen::Vector2d>& astar_path,
        const TrajectoryParams& params = TrajectoryParams()
    ):
        map_(map),
        astar_path_(astar_path),
        params_(params),
        in_opt_(false) {
        preprocessAstarPath();
    }
    auto set_map(std::shared_ptr<grid_map::GridMap> map) -> void {
        map_ = map;
    }
    auto set_astar_path(const std::vector<Eigen::Vector2d>& astar_path) -> void {
        astar_path_ = astar_path;
    }
    auto set_params(const TrajectoryParams& params = TrajectoryParams()) -> void {
        params_ = params;
    }
    auto set_start_vel(const Eigen::Vector2d& start_vel) -> void {
        // 路径首段方向
        Eigen::Vector2d dir = astar_path_[1] - astar_path_[0];
        const double dir_len = dir.norm();
        if (dir_len < 1e-6) {
            init_cond.col(1) = start_vel.norm() > 0.01 ? start_vel : Eigen::Vector2d::Zero();
            return;
        }
        dir /= dir_len;
        if (start_vel.norm() > 0.01) {
            // 把实际速度投影到路径首段方向：
            //   方向一致 → 保留沿路径分量（丢弃横向分量，避免样条侧向扭曲）；
            //   方向相反（换目标/掉头）→ 退化为沿路径方向小速度重新起步，
            //   避免样条被迫以高速掉头导致能量爆炸、线搜索失败（-1009）
            const double v_along = start_vel.dot(dir);
            init_cond.col(1) = (v_along > 0.01) ? dir * v_along : dir * 0.5;
        } else {
            // 静止时沿路径方向起步
            init_cond.col(1) = dir * 0.5;
        }
    }
    auto set_end_vel(const Eigen::Vector2d& end_vel) -> void {
        if (end_vel.norm() > 0.01) {
            end_cond.col(1) = end_vel;
        } else {
            end_cond.col(1) = (astar_path_.back() - astar_path_[astar_path_.size() - 2]).normalized() * 0.1;
        }
    }
    bool plan() {
        if (astar_path_.size() < 2) return false;
        // 准备初始条件
        // Prepare initial conditions

        init_cond.col(0) = astar_path_.front();
        end_cond.col(0) = astar_path_.back();
        // init_cond.col(1) = (astar_path_[1] - astar_path_[0]).normalized() * 0.1;
        // end_cond.col(1) = (astar_path_.back() - astar_path_[astar_path_.size() - 2]).normalized() * 0.1;
        // 轨迹总长度（米）
        double total_len = params_.total_len;
        //样条数
        int piece_num = (int)(total_len / params_.piece_len);
        if (piece_num < 2) {
            piece_num = 2;
        }
        params_.piece_len = total_len / piece_num;

        // Sample intermediate control points from A* path
        // 从A*路径采样的中间控制点
        //中间点
        Eigen::MatrixXd inner_pos(2, piece_num - 1);

        std::vector<Eigen::Vector2d> inner_pos_node;

        double step_len = total_len / piece_num;

        double accumulated_len = 0.0;

        for (int i = 1; i < piece_num; ++i) {
            double target_len = i * step_len;

            while (accumulated_len < target_len && current_segment_ < astar_path_.size() - 1) {
                double seg_len = (astar_path_[current_segment_ + 1] - astar_path_[current_segment_]).norm();

                if (accumulated_len + seg_len >= target_len) {
                    double ratio = (target_len - accumulated_len) / seg_len;

                    Eigen::Vector2d point = astar_path_[current_segment_]
                        + ratio * (astar_path_[current_segment_ + 1] - astar_path_[current_segment_]);

                    inner_pos_node.push_back(point);

                    break;
                }
                accumulated_len += seg_len;
                current_segment_++;
            }
        }

        inner_pos.resize(2, inner_pos_node.size());
        for (size_t i = 0; i < inner_pos_node.size(); i++) {
            inner_pos.col(i) = inner_pos_node[i];
        }

        double total_time = params_.total_time;
        int result = optimizeSE2Traj(init_cond, inner_pos, end_cond, total_time);
        // L-BFGS 返回码：0=收敛(LBFGS_CONVERGENCE)、1=满足停止准则(LBFGS_STOP)均为成功，
        // 负数才是错误。不能直接 return result（int→bool 会把 0 转成 false，
        // 导致“初始猜测已是驻点/正常收敛”被误判为失败——到 goal 附近时必现）。
        return result >= 0;
    }

    PPoly2D getOptimizedTrajectory() const {
        return trajectory_;
    }

    struct TrajectoryMetrics {
        double max_velocity;
        double min_clearance;
        double total_time;
        double trajectory_energy;
        double path_deviation;
    };

    TrajectoryMetrics evaluateTrajectory() const {
        TrajectoryMetrics metrics;
        metrics.max_velocity = 0.0;
        metrics.min_clearance = std::numeric_limits<double>::max();
        metrics.total_time = trajectory_.getDuration();
        metrics.trajectory_energy = 0.0;
        metrics.path_deviation = 0.0;

        if (!trajectory_.isInitialized()) return metrics;

        const double dt = 0.01;
        int sample_count = 0;
        for (double t = 0.0; t < metrics.total_time; t += dt) {
            Vector2d pos = trajectory_.evaluate(t, 0);
            Vector2d vel = trajectory_.evaluate(t, 1);
            Vector2d acc = trajectory_.evaluate(t, 2);

            metrics.trajectory_energy += acc.squaredNorm() * dt;
            metrics.max_velocity = std::max(metrics.max_velocity, vel.norm());
            metrics.min_clearance = std::min(metrics.min_clearance, map_->getDistance(pos));

            double min_dist = std::numeric_limits<double>::max();
            for (const auto& astar_pt: astar_path_) {
                min_dist = std::min(min_dist, (pos - astar_pt).norm());
            }
            metrics.path_deviation += min_dist;
            sample_count++;
        }

        if (sample_count > 0) {
            metrics.path_deviation /= sample_count;
        }

        return metrics;
    }

    std::vector<Eigen::Vector2d> sampleTrajectory(double dt = 0.1) const {
        std::vector<Eigen::Vector2d> path;
        if (!trajectory_.isInitialized()) return path;

        const double total_time = trajectory_.getDuration();
        for (double t = 0.0; t <= total_time; t += dt) {
            path.push_back(trajectory_.evaluate(t, 0));
        }

        if (!path.empty() && path.back() != trajectory_.evaluate(total_time, 0)) {
            path.push_back(trajectory_.evaluate(total_time, 0));
        }

        return path;
    }

private:
    void preprocessAstarPath() {
        // Prepare A* path for fast nearest neighbor queries
    }

    int
    optimizeSE2Traj(const MatrixXd& initPos, const MatrixXd& innerPtsPos, const MatrixXd& endPos, double totalTime) {
        in_opt_ = true;
        piece_pos_ = innerPtsPos.cols() + 1;

        init_pos_ = initPos;
        end_pos_ = endPos;

        int variable_num = 2 * (piece_pos_ - 1) + 1;
        dim_T = 1;

        Eigen::VectorXd x;
        x.resize(variable_num);

        double& tau = x(0);
        Eigen::Map<Eigen::MatrixXd> Ppos(x.data() + dim_T, 2, piece_pos_ - 1);

        tau = logC2(totalTime);
        Ppos = innerPtsPos;

        Eigen::VectorXd Tpos;
        Tpos.resize(piece_pos_);
        calTfromTauUni(tau, Tpos);

        generateTrajectory(initPos, endPos, Ppos, Tpos);

        auto metrics = evaluateTrajectory();
        // 诊断：初始轨迹是否含 NaN（NaN 会让线搜索立即失败 LBFGSERR_INVALID_FUNCVAL）
        {
            const Eigen::Vector2d p0 = trajectory_.evaluate(0.0, 0);
            const Eigen::Vector2d p1 = trajectory_.evaluate(totalTime * 0.5, 0);
            const Eigen::Vector2d v0 = trajectory_.evaluate(0.0, 1);
            if (!p0.allFinite() || !p1.allFinite() || !v0.allFinite() || !std::isfinite(metrics.max_velocity)) {
                spdlog::warn(
                    "[traj_opt] initial trajectory NaN/Inf! total_time={:.4f} pieces={} inner={} max_v={}",
                    totalTime, piece_pos_, static_cast<int>(innerPtsPos.cols()), metrics.max_velocity
                );
            }
        }

        lbfgs::lbfgs_parameter_t lbfgs_params;
        lbfgs_params.mem_size = params_.mem_size;
        lbfgs_params.past = params_.past;
        lbfgs_params.g_epsilon = params_.g_epsilon;
        lbfgs_params.min_step = params_.min_step;
        lbfgs_params.delta = params_.delta;
        lbfgs_params.max_iterations = params_.max_iter;

        double final_cost;
        int result = lbfgs::lbfgs_optimize(
            x,
            final_cost,
            [](void* instance, const Eigen::VectorXd& x, Eigen::VectorXd& grad) {
                return static_cast<TrajectoryOptimizer*>(instance)->costFunction(x, grad);
            },
            nullptr,
            nullptr,
            this,
            lbfgs_params
        );

        if (result < 0) {
            spdlog::warn(
                "[traj_opt] L-BFGS failed: {} (code {}) initial_cost={:.4f} final_cost={:.4f}",
                lbfgs::lbfgs_strerror(result), result, metrics.trajectory_energy, final_cost
            );
        }
        in_opt_ = false;
        return result;
    }

    double costFunction(const Eigen::VectorXd& x, Eigen::VectorXd& grad) {
        double cost = 0.0;

        const double& tau = x(0);
        double& grad_tau = grad(0);
        Eigen::Map<const Eigen::MatrixXd> Ppos(x.data() + dim_T, 2, piece_pos_ - 1);
        Eigen::Map<Eigen::MatrixXd> gradPpos(grad.data() + dim_T, 2, piece_pos_ - 1);

        Eigen::VectorXd Tpos;
        Tpos.resize(piece_pos_);
        calTfromTauUni(tau, Tpos);
        generateTrajectory(init_pos_, end_pos_, Ppos, Tpos);

        double constrain_cost = 0.0;
        Eigen::MatrixXd gdCpos_constrain;
        Eigen::VectorXd gdTpos_constrain;
        calculateConstraintCostGrad(trajectory_, constrain_cost, gdCpos_constrain, gdTpos_constrain);

        Eigen::MatrixXd gradPpos_constrain;
        Eigen::VectorXd gradTpos_constrain;
        calGradCTtoQT(gdCpos_constrain, gdTpos_constrain, gradPpos_constrain, gradTpos_constrain);

        double energy = cubic_spline_.getEnergy();
        double energy_cost = params_.rho_energy * energy;

        CubicSpline2D::MatrixType gradP_energy = cubic_spline_.getEnergyGradInnerP();
        Eigen::VectorXd gradT_energy = cubic_spline_.getEnergyGradTimes();

        gradPpos = gradPpos_constrain + params_.rho_energy * gradP_energy.transpose();
        Eigen::VectorXd gradTpos_total = gradTpos_constrain + params_.rho_energy * gradT_energy;

        double tau_cost = params_.rho_T * expC2(tau);
        double grad_Tsum = params_.rho_T + gradTpos_total.sum() / piece_pos_;
        grad_tau = grad_Tsum * getTtoTauGrad(tau);

        cost = constrain_cost + energy_cost + tau_cost;
        return cost;
    }

    void calculateConstraintCostGrad(PPoly2D& traj, double& cost, Eigen::MatrixXd& gdCpos, Eigen::VectorXd& gdTpos) {
        cost = 0.0;
        double v_cost = 0.0;
        double occ_cost = 0.0;
        double path_cost = 0.0;

        const int N = traj.getNumSegments();
        gdCpos.resize(4 * N, 2);
        gdCpos.setZero();
        gdTpos.resize(N);
        gdTpos.setZero();

        const auto& breaks = traj.getBreakpoints();
        const MatrixXd& coeffs = traj.getCoefficients();

        Eigen::Vector2d pos, vel, acc;
        double grad_time = 0.0;
        Eigen::Vector2d grad_p = Eigen::Vector2d::Zero();
        Eigen::Vector2d grad_v = Eigen::Vector2d::Zero();
        Eigen::Vector2d grad_sdf = Eigen::Vector2d::Zero();
        Eigen::Matrix<double, 4, 1> beta0, beta1, beta2;
        double s1, s2, s3;
        double step, alpha, omg;

        for (int i = 0; i < N; ++i) {
            const Eigen::Matrix<double, 4, 2>& c = coeffs.block<4, 2>(i * 4, 0);
            step = (breaks[i + 1] - breaks[i]) / params_.int_K;
            s1 = 0.0;

            for (int j = 0; j <= params_.int_K; ++j) {
                alpha = 1.0 / params_.int_K * j;
                grad_p.setZero();
                grad_v.setZero();
                grad_sdf.setZero();
                grad_time = 0.0;

                s2 = s1 * s1;
                s3 = s2 * s1;
                beta0 << 1.0, s1, s2, s3;
                beta1 << 0.0, 1.0, 2.0 * s1, 3.0 * s2;
                beta2 << 0.0, 0.0, 2.0, 6.0 * s1;
                pos = c.transpose() * beta0;
                vel = c.transpose() * beta1;
                acc = c.transpose() * beta2;

                omg = (j == 0 || j == params_.int_K) ? 0.5 : 1.0;

                // 1. Velocity constraint
                double vxy_snorm = vel.squaredNorm();
                double vViola = vxy_snorm - params_.max_v * params_.max_v;
                if (vViola > 0) {
                    grad_v += params_.rho_v * 6 * vViola * vViola * vel;
                    double cost_v = params_.rho_v * vViola * vViola * vViola;
                    cost += cost_v * omg * step;
                    v_cost += cost_v * omg * step;
                    grad_time += omg * (cost_v / params_.int_K + step * alpha * grad_v.dot(acc));
                }

                // 1.5 巡航速度项：速度低于 v_des 时惩罚，推动起步段快速提速到期望速度。
                //     代价 rho_v_des*(v_des-|v|)^2，梯度 -2*rho_v_des*(v_des-|v|)*v/|v|
                //     （v=0 处梯度取 0，避免除零；代价本身仍会推离低速）
                if (params_.rho_v_des > 0.0) {
                    const double v_norm = std::sqrt(vxy_snorm);
                    const double v_des = params_.max_v * params_.v_des_ratio;
                    if (v_norm < v_des) {
                        const double viola = v_des - v_norm;
                        const double cost_vdes = params_.rho_v_des * viola * viola;
                        Eigen::Vector2d grad_vdes = Eigen::Vector2d::Zero();
                        if (v_norm > 1e-6) {
                            grad_vdes = params_.rho_v_des * (-2.0 * viola) * (vel / v_norm);
                        }
                        grad_v += grad_vdes;
                        cost += cost_vdes * omg * step;
                        v_cost += cost_vdes * omg * step;
                        grad_time += omg * (cost_vdes / params_.int_K + step * alpha * grad_vdes.dot(acc));
                    }
                }

                // 2. Collision constraint
                double sdf_value;
                map_->getDistanceAndGradient(pos, sdf_value, grad_sdf);
                double cViola = params_.safe_threshold - sdf_value;
                if (cViola > 0 && sdf_value < 5) {
                    double penalty;
                    Eigen::Vector2d grad_pc;

                    if (cViola < 0.1) {
                        penalty = cViola * cViola;
                        grad_pc = -2.0 * cViola * grad_sdf;
                    } else {
                        penalty = cViola;
                        grad_pc = -grad_sdf;
                    }

                    double cost_c = params_.rho_collision * penalty;
                    Eigen::Vector2d grad_pc_scaled = params_.rho_collision * grad_pc;

                    cost += cost_c * omg * step;
                    occ_cost += cost_c * omg * step;
                    grad_time += omg * (cost_c / params_.int_K + step * alpha * grad_pc_scaled.dot(vel));
                    grad_p += grad_pc_scaled;
                }

                gdCpos.block<4, 2>(i * 4, 0) += (beta0 * grad_p.transpose() + beta1 * grad_v.transpose()) * omg * step;
                gdTpos(i) += grad_time;

                s1 += step;
            }
        }

        //std::cout << "Cost breakdown - Vel: " << v_cost << ", Coll: " << occ_cost << std::endl;
    }

    void calGradCTtoQT(
        const Eigen::MatrixXd& gdCpos,
        const Eigen::VectorXd& gdTpos,
        Eigen::MatrixXd& gradPpos,
        Eigen::VectorXd& gradTpos_out
    ) {
        CubicSpline2D::MatrixType gdC_typed = gdCpos;
        CubicSpline2D::MatrixType gradByPoints;
        Eigen::VectorXd gradByTimes;

        cubic_spline_.propagateGrad(gdC_typed, gdTpos, gradByPoints, gradByTimes);

        gradPpos = gradByPoints.transpose();
        gradTpos_out = gradByTimes;

        // std::cout << "Gradient norm - Ppos: " << gradPpos.norm()
        //         << ", Tpos: " << gradTpos_out.norm() << std::endl;
    }

    void generateTrajectory(
        const MatrixXd& initPos,
        const MatrixXd& endPos,
        const MatrixXd& innerPts,
        Eigen::VectorXd Tpos
    ) {
        std::vector<double> times;
        times.reserve(piece_pos_ + 1);

        double t = 0;
        for (int i = 0; i < Tpos.size(); ++i) {
            times.push_back(t);
            t += Tpos(i);
        }
        times.push_back(t);

        SplineTrajectory::SplineVector2D waypoints;
        waypoints.push_back(initPos.col(0));
        for (int i = 0; i < innerPts.cols(); ++i) {
            waypoints.push_back(innerPts.col(i));
        }
        waypoints.push_back(endPos.col(0));

        SplineTrajectory::BoundaryConditions<2> bc;
        bc.start_velocity = initPos.col(1);
        bc.end_velocity = endPos.col(1);

        cubic_spline_.update(times, waypoints, bc);
        trajectory_ = cubic_spline_.getTrajectory();
    }

    double calculatePathLength(const std::vector<Eigen::Vector2d>& path) {
        double length = 0.0;
        for (size_t i = 1; i < path.size(); ++i) {
            length += (path[i] - path[i - 1]).norm();
        }
        return length;
    }

    inline double logC2(const double& T) {
        return T > 1.0 ? (sqrt(2.0 * T - 1.0) - 1.0) : (1.0 - sqrt(2.0 / T - 1.0));
    }

    inline void calTfromTauUni(const double& tau, Eigen::VectorXd& T) {
        T.setConstant(expC2(tau) / T.size());
    }

    inline double expC2(const double& tau) {
        return tau > 0.0 ? ((0.5 * tau + 1.0) * tau + 1.0) : 1.0 / ((0.5 * tau - 1.0) * tau + 1.0);
    }

    inline double getTtoTauGrad(const double& tau) {
        if (tau > 0) return tau + 1.0;
        else {
            double denSqrt = (0.5 * tau - 1.0) * tau + 1.0;
            return (1.0 - tau) / (denSqrt * denSqrt);
        }
    }

    std::shared_ptr<grid_map::GridMap> map_;
    std::vector<Eigen::Vector2d> astar_path_;
    TrajectoryParams params_;
    bool in_opt_;
    int piece_pos_;
    int dim_T;
    MatrixXd init_pos_;
    MatrixXd end_pos_;
    PPoly2D trajectory_;
    CubicSpline2D cubic_spline_;
    int current_segment_ = 0;
    //vel
    Eigen::Matrix2d init_cond, end_cond;
};

} // namespace TrajOpt

#endif // TRAJ_OPT_HPP