#pragma once
#include "gcopter/lbfgs.hpp"
namespace planner {
        struct Config
    {
        // kinematic constraints
        // 运动学约束
        double max_vel_;
        double min_vel_;
        double max_acc_;
        double max_omega_;
        double max_domega_;
        double max_centripetal_acc_;

        bool if_directly_constrain_v_omega_;
    };
    struct PenaltyWeights{
        double time_weight;
        double time_weight_backup_for_replan;
        double acc_weight;
        double domega_weight;
        double collision_weight;
        double moment_weight;
        double mean_time_weight;
        double cen_acc_weight;
    };

    // For trajectory pre-processing
    struct PathpenaltyWeights{
        double time_weight;
        double bigpath_sdf_weight;
        double mean_time_weight;
        double moment_weight;
        double acc_weight;
        double domega_weight;
    };

    // For trajectory pre-processing
    struct PathLbfgsParams{
        lbfgs::lbfgs_parameter_t path_lbfgs_params;
        double normal_past;
        double shot_path_past;
        double shot_path_horizon;
    };
}