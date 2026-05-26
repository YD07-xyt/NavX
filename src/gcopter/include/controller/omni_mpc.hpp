#ifndef MPC_H
#define MPC_H

#include <vector>
#include <Eigen/Dense>
#include<spdlog/spdlog.h>
#include <casadi/casadi.hpp>
#include <chrono>


class OmniMpc
{
private:
    //mpc params
    int N_;  //horizon
    double dt_;  //step
    //constrains
    double u_max_, u_min_;
    double w_max_, w_min_;
    
    //weights
    casadi::DM Q_, R_;
    casadi::MX X;
    casadi::MX U;
    
    casadi::Function kinematic_equation_;
    //OptiSol solution_; 报错没有默认构造函数
    std::unique_ptr<casadi::OptiSol> solution_;
    

public:
    OmniMpc();
    ~OmniMpc();

    casadi::Function setKinematicEquation();
    void setWeights(std::vector<double> weights);
    bool solve(Eigen::Vector3d current_states, Eigen::MatrixXd desired_states);
    std::vector<double> getFirstU();
    std::vector<double> getPredictX();
};

#endif