#include "../include/controller/omni_mpc.hpp"
#include <spdlog/spdlog.h>

OmniMpc::OmniMpc() {
    N_ = 10;
    dt_ = 0.1;
    u_max_ = 4;
    w_max_ = 2;
    // 权重顺序：Qx, Qy, Qtheta, Rvx, Rvy, Romega
    std::vector<double> weights = {10, 10, 1, 1, 1, 0.5}; // Q,R
    u_min_ = -u_max_;
    w_min_ = -w_max_;
    
    Q_ = casadi::DM::zeros(3, 3);
    R_ = casadi::DM::zeros(3, 3);   // 控制量变为3维
    
    setWeights(weights);
    kinematic_equation_ = setKinematicEquation();
}

OmniMpc::~OmniMpc() {}

void OmniMpc::setWeights(std::vector<double> weights) {
    Q_(0, 0) = weights[0];
    Q_(1, 1) = weights[1];
    Q_(2, 2) = weights[2];
    R_(0, 0) = weights[3];
    R_(1, 1) = weights[4];
    R_(2, 2) = weights[5];
}

casadi::Function OmniMpc::setKinematicEquation() {
    // 状态：x, y, theta
    casadi::MX x = casadi::MX::sym("x");
    casadi::MX y = casadi::MX::sym("y");
    casadi::MX theta = casadi::MX::sym("theta");
    casadi::MX state_vars = casadi::MX::vertcat({x, y, theta});

    // 控制：车体纵向速度 u，横向速度 v，角速度 w
    casadi::MX u = casadi::MX::sym("u");
    casadi::MX v = casadi::MX::sym("v");
    casadi::MX w = casadi::MX::sym("w");
    casadi::MX control_vars = casadi::MX::vertcat({u, v, w});

    // 全向运动学：世界坐标系下的导数
    casadi::MX rhs = u * casadi::MX::cos(theta) - v * casadi::MX::sin(theta);
    rhs = casadi::MX::vertcat({rhs,
                               u * casadi::MX::sin(theta) + v * casadi::MX::cos(theta),
                               w});
    return casadi::Function("kinematic_equation", {state_vars, control_vars}, {rhs});
}

bool OmniMpc::solve(Eigen::Vector3d current_states, Eigen::MatrixXd desired_states) {
    casadi::Opti opti = casadi::Opti();
    casadi::Slice all;

    // 优化变量：状态 X (3 x N+1)，控制 U (3 x N)
    X = opti.variable(3, N_ + 1);
    U = opti.variable(3, N_);
    casadi::MX x = X(0, all);
    casadi::MX y = X(1, all);
    casadi::MX theta = X(2, all);
    casadi::MX u = U(0, all);   // 纵向速度
    casadi::MX v = U(1, all);   // 横向速度
    casadi::MX w = U(2, all);   // 角速度

    // 参数：参考轨迹和当前状态
    casadi::MX X_ref = opti.parameter(3, N_ + 1);
    casadi::MX X_cur = opti.parameter(3);
    casadi::DM x_tmp1 = {current_states[0], current_states[1], current_states[2]};
    opti.set_value(X_cur, x_tmp1);
    //spdlog::info("set current state success");

    // 设置参考轨迹
    std::vector<double> X_ref_v(desired_states.data(), desired_states.data() + desired_states.size());
    casadi::DM X_ref_d(X_ref_v);
    X_ref = casadi::MX::reshape(X_ref_d, 3, N_ + 1);

    // 代价函数
    casadi::MX cost = 0;
    for (int i = 0; i < N_; ++i) {
        casadi::MX X_err = X(all, i) - X_ref(all, i);
        casadi::MX U_0 = U(all, i);
        cost += casadi::MX::mtimes({X_err.T(), Q_, X_err});
        cost += casadi::MX::mtimes({U_0.T(), R_, U_0});
    }
    cost += casadi::MX::mtimes({(X(all, N_) - X_ref(all, N_)).T(), Q_,
                                X(all, N_) - X_ref(all, N_)});
    opti.minimize(cost);

    // 动力学约束（全向模型）
    for (int i = 0; i < N_; ++i) {
        std::vector<casadi::MX> input(2);
        input[0] = X(all, i);
        input[1] = U(all, i);
        casadi::MX X_next = kinematic_equation_(input)[0] * dt_ + X(all, i);
        opti.subject_to(X_next == X(all, i + 1));
    }

    // 初始状态约束
    opti.subject_to(X(all, 0) == X_cur);

    // 控制量约束（对称限幅）
    opti.subject_to(-u_max_ <= u <= u_max_);
    opti.subject_to(-u_max_ <= v <= u_max_);
    opti.subject_to(-w_max_ <= w <= w_max_);

    // 求解器配置
    casadi::Dict solver_opts;
    solver_opts["expand"] = true;
    solver_opts["ipopt.max_iter"] = 100;
    solver_opts["ipopt.print_level"] = 0;
    solver_opts["print_time"] = 0;
    solver_opts["ipopt.acceptable_tol"] = 1e-6;
    solver_opts["ipopt.acceptable_obj_change_tol"] = 1e-6;

    opti.solver("ipopt", solver_opts);
    solution_ = std::make_unique<casadi::OptiSol>(opti.solve());
    return true;
}

std::vector<double> OmniMpc::getFirstU() {
    std::vector<double> res;
    // 顺序：vx, vy, omega
    auto first_u = solution_->value(U)(0, 0);
    auto first_v = solution_->value(U)(1, 0);
    auto first_w = solution_->value(U)(2, 0);
    res.push_back(static_cast<double>(first_u));
    res.push_back(static_cast<double>(first_v));
    res.push_back(static_cast<double>(first_w));
    return res;
}

std::vector<double> OmniMpc::getPredictX() {
    std::vector<double> res;
    auto predict_x = solution_->value(X);
    spdlog::info("nomal");
    for (int i = 0; i <= N_; ++i) {
        res.push_back(static_cast<double>(predict_x(0, i)));
        res.push_back(static_cast<double>(predict_x(1, i)));
    }
    return res;
}