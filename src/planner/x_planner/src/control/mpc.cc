#include"../../include/control/mpc.h"
#include <cppad/ipopt/solve.hpp>

using CppAD::AD;
namespace planner {


Mpc::Mpc() {}
Mpc::~Mpc() {}

std::vector<double> Mpc::solve(Eigen::Vector3d state, Eigen::MatrixXd desired_state) {
    bool ok = true;
    typedef CPPAD_TESTVECTOR(double) Dvector;

    double x = state(0);
    double y = state(1);
    double psi = state(2);

    int n_var = mpc_config.N * 3 + (mpc_config.N - 1) * 3;
    int n_constraints = mpc_config.N * 3;

    Dvector vars(n_var);
    for (int i = 0; i < n_var; ++i) {
        vars[i] = 0.0;
    }

    Dvector vars_lowerbound(n_var);
    Dvector vars_upperbound(n_var);

    for (int i = 0; i < u_start; ++i) {
        vars_lowerbound[i] = -1e19;
        vars_upperbound[i] = 1e19;
    }
    for (int i = u_start; i < v_start; ++i) {
        vars_lowerbound[i] = -mpc_config.u_max; //前进速度 u -
        vars_upperbound[i] = mpc_config.u_max;
    }
    for (int i = v_start; i < r_start; ++i) {
        vars_lowerbound[i] = -mpc_config.v_max;
        vars_upperbound[i] = mpc_config.v_max;        
    }
    for (int i = r_start; i < n_var; ++i) {
        vars_lowerbound[i] = -mpc_config.r_max;
        vars_upperbound[i] = mpc_config.r_max;
    }

    Dvector constraints_lowerbound(n_constraints);
    Dvector constraints_upperbound(n_constraints);

    for (int i = 0; i < n_constraints; ++i) {
        constraints_lowerbound[i] = 0;
        constraints_upperbound[i] = 0;
    }

    constraints_lowerbound[x_start] = x;
    constraints_lowerbound[y_start] = y;
    constraints_lowerbound[psi_start] = psi;
    constraints_upperbound[x_start] = x;
    constraints_upperbound[y_start] = y;
    constraints_upperbound[psi_start] = psi;

    FG_eval fg_eval(desired_state);

    std::string options;
    options += "Integer print_level 0\n";
    options += "Sparse true forward\n";
    options += "Sparse true reverse\n";
    options += "Numeric max_cpu_time 0.5\n";

    CppAD::ipopt::solve_result<Dvector> solution;

    CppAD::ipopt::solve<Dvector, FG_eval> (
        options, vars, vars_lowerbound, vars_upperbound, constraints_lowerbound,
        constraints_upperbound, fg_eval, solution
    );

    ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;

    auto cost = solution.obj_value;
    //cout << "cost:" << cost << " ";

    std::vector<double> solved;
    solved.push_back(solution.x[u_start]);
    solved.push_back(solution.x[v_start]);
    for (int i = 0; i < mpc_config.N; ++i) {
        solved.push_back(solution.x[x_start + i]);
        solved.push_back(solution.x[y_start + i]);
    }
    std::cout << "u:" << solution.x[u_start] << " " << "v:" << solution.x[v_start] << std::endl;
    return solved;
}
}