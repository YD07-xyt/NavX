#ifndef MPC_H
#define MPC_H
#include <cppad/ipopt/solve.hpp>
#include <vector>
#include "Eigen/Eigen"
using CppAD::AD;

namespace planner {

struct config{
    int x_weight = 10;
    int y_weight = 10;
    int psi_weight =0;
    int u_weight = 1;
    int v_weight = 1;
    int N = 30;      // 预测步数
    double dt = 0.1; // 控制周期（秒）
    double u_max = 3.2;
    double v_max= 3.2; //横向速度 v -
    double r_max =1;// rad/s 航角速度 r
}mpc_config;


int x_start = 0;
int y_start = x_start + mpc_config.N;
int psi_start = y_start + mpc_config.N;
int u_start = psi_start + mpc_config.N;
int v_start = u_start + mpc_config.N - 1;
int r_start = v_start + mpc_config.N - 1;
double num = 0;



class Mpc
{
private:
    
public:
    Mpc(/* args */);
    ~Mpc();
    
    std::vector<double> solve(Eigen::Vector3d state, Eigen::MatrixXd desired_state);
};
class FG_eval
{
public:
    Eigen::MatrixXd desired_state_;
    
    FG_eval(Eigen::MatrixXd desired_state) {
        desired_state_ = desired_state;
    }
    typedef CPPAD_TESTVECTOR(AD<double>) ADvector;
    
    void operator()(ADvector &fg, const ADvector &vars) {
        //cost
        fg[0] = 0;

        //weights
        const int x_weight = 10;
        const int y_weight = 10;
        const int psi_weight =0;
        const int u_weight = 1;
        const int v_weight = 1;

        for (int i = 0; i < mpc_config.N; ++i) {
            AD<double> x_des = desired_state_(i, 0);
            AD<double> y_des = desired_state_(i, 1);
            AD<double> psi_des = desired_state_(i, 2);
            fg[0] += x_weight * CppAD::pow((vars[x_start + i] - x_des), 2);
            fg[0] += y_weight * CppAD::pow((vars[y_start + i] - y_des), 2);
            if (psi_des > M_PI) psi_des -= 2 * M_PI;
            if (psi_des < -M_PI) psi_des += 2 * M_PI;
            fg[0] += psi_weight * CppAD::pow(vars[psi_start + i] - psi_des + M_PI / 2, 2);
        }
        for (int i = 0; i < mpc_config.N-1; ++i) {
            fg[0] += u_weight * CppAD::pow(vars[u_start + i ] , 2);
            fg[0] += v_weight * CppAD::pow(vars[v_start + i ] , 2);
        }

        fg[1 + x_start] = vars[x_start];
        fg[1 + y_start] = vars[y_start];
        fg[1 + psi_start] = vars[psi_start];
        
        for (int i = 1; i < mpc_config.N; ++i) {
            AD<double> x_1 = vars[x_start + i];
            AD<double> y_1 = vars[y_start + i];
            AD<double> psi_1 = vars[psi_start + i];

            AD<double> x_0 = vars[x_start + i - 1];
            AD<double> y_0 = vars[y_start + i - 1];
            AD<double> psi_0 = vars[psi_start + i - 1];

            AD<double> u_0 = vars[u_start + i - 1];
            AD<double> v_0 = vars[v_start + i - 1];
            AD<double> r_0 = vars[r_start + i - 1];

            fg[1 + x_start + i] = x_1 - (x_0 + (u_0 * CppAD::cos(psi_0) - v_0 * CppAD::sin(psi_0)) * mpc_config.dt);
            fg[1 + y_start + i] = y_1 - (y_0 + (u_0 * CppAD::sin(psi_0) + v_0 * CppAD::cos(psi_0)) * mpc_config.dt);
            fg[1 + psi_start + i] = psi_1 - (psi_0 + r_0 * mpc_config.dt);
        }
        
    }
     
};

}

#endif
