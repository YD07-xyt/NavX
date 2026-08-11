#pragma once

#include "utils/logger.hpp"
#include <cmath>
#include <controller/omni_lmpc.hpp>
#include <fsm/fsm.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>
namespace planner {

struct Config {
  std::string mapTopic = "/terrain_map_ext";
  std::string targetTopic = "/goal_pose";
  std::string odomTopic = "/fake_odom";
  double map_size = 43.0;
  double resolution = 0.1;
  bool mapping_model; // true: 建图模式，false: 规划模式
  std::string map_params_path = "/home/xyt/map/src/gcopter/map/global_map.pgm";
  FSM::FSMConfig fsm_config;
  controller::LMpc::LMpcParam lmpc_param;

  Config(const rclcpp::Node::SharedPtr &node) {
    // ========== 1. 声明所有参数（含默认值） ==========
    // 已有参数
    node->declare_parameter<std::string>("map_topic", mapTopic);
    node->declare_parameter<std::string>("target_topic", targetTopic);
    node->declare_parameter<std::string>("odom_topic", odomTopic);
    node->declare_parameter<double>("map_size", map_size);
    node->declare_parameter<double>("resolution", resolution);
    node->declare_parameter<bool>("mapping_model", mapping_model);
    node->declare_parameter<std::string>("map_params_path", map_params_path);
    // AstarParam 剩余参数
    node->declare_parameter<double>("max_vel",
                                    fsm_config.astar_param_.max_vel_);
    node->declare_parameter<double>("max_acc",
                                    fsm_config.astar_param_.max_acc_);
    node->declare_parameter<double>("time_resolution",
                                    fsm_config.astar_param_.time_resolution_);
    node->declare_parameter<int>("min_traj_num",
                                 fsm_config.astar_param_.min_traj_num_);
    node->declare_parameter<double>("traj_cut_length",
                                    fsm_config.astar_param_.traj_cut_length_);
    node->declare_parameter<double>("distance_weight",
                                    fsm_config.astar_param_.distance_weight_);
    node->declare_parameter<double>("yaw_weight",
                                    fsm_config.astar_param_.yaw_weight_);

    // FSM::FSMConfig 参数
    node->declare_parameter<double>("fsm.safe_threshold",
                                    fsm_config.safe_threshold_);
    node->declare_parameter<double>("fsm.deviation.x",
                                    fsm_config.deviation.x());
    node->declare_parameter<double>("fsm.deviation.y",
                                    fsm_config.deviation.y());
    node->declare_parameter<double>("fsm.deviation.z",
                                    fsm_config.deviation.z());
    node->declare_parameter<double>("fsm.replan_interval",
                                    fsm_config.replan_interval_);
    node->declare_parameter<double>("fsm.replan_lateral_dev",
                                    fsm_config.replan_lateral_dev_);
    node->declare_parameter<double>("fsm.min_replan_interval",
                                    fsm_config.min_replan_interval_);
    node->declare_parameter<double>("fsm.goal_reached_radius",
                                    fsm_config.goal_reached_radius_);

    // TrajectoryParams 参数（嵌套在 fsm_config_.params_ 中）
    node->declare_parameter<double>("traj.rho_v", fsm_config.params_.rho_v);
    node->declare_parameter<double>("traj.rho_collision",
                                    fsm_config.params_.rho_collision);
    node->declare_parameter<double>("traj.rho_T", fsm_config.params_.rho_T);
    node->declare_parameter<double>("traj.rho_energy",
                                    fsm_config.params_.rho_energy);
    node->declare_parameter<double>("traj.max_v", fsm_config.params_.max_v);
    node->declare_parameter<double>("traj.rho_v_des", fsm_config.params_.rho_v_des);
    node->declare_parameter<double>("traj.v_des_ratio", fsm_config.params_.v_des_ratio);
    node->declare_parameter<double>("traj.safe_threshold",
                                    fsm_config.params_.safe_threshold);
    node->declare_parameter<int>("traj.int_K", fsm_config.params_.int_K);
    node->declare_parameter<int>("traj.mem_size", fsm_config.params_.mem_size);
    node->declare_parameter<int>("traj.past", fsm_config.params_.past);
    node->declare_parameter<double>("traj.g_epsilon",
                                    fsm_config.params_.g_epsilon);
    node->declare_parameter<double>("traj.min_step",
                                    fsm_config.params_.min_step);
    node->declare_parameter<double>("traj.delta", fsm_config.params_.delta);
    node->declare_parameter<int>("traj.max_iter", fsm_config.params_.max_iter);

    // ========== 2. 读取所有参数（覆盖成员变量） ==========
    node->get_parameter("map_topic", mapTopic);
    node->get_parameter("target_topic", targetTopic);
    node->get_parameter("odom_topic", odomTopic);
    node->get_parameter("map_size", map_size);
    node->get_parameter("resolution", resolution);
    node->get_parameter("mapping_model", mapping_model);
    node->get_parameter("map_params_path", map_params_path);
    node->get_parameter("max_vel", fsm_config.astar_param_.max_vel_);
    node->get_parameter("max_acc", fsm_config.astar_param_.max_acc_);
    node->get_parameter("time_resolution",
                        fsm_config.astar_param_.time_resolution_);
    node->get_parameter("min_traj_num", fsm_config.astar_param_.min_traj_num_);
    node->get_parameter("traj_cut_length",
                        fsm_config.astar_param_.traj_cut_length_);
    node->get_parameter("distance_weight",
                        fsm_config.astar_param_.distance_weight_);
    node->get_parameter("yaw_weight", fsm_config.astar_param_.yaw_weight_);

    node->get_parameter("fsm.safe_threshold", fsm_config.safe_threshold_);
    double dev_x, dev_y, dev_z;
    node->get_parameter("fsm.deviation.x", dev_x);
    node->get_parameter("fsm.deviation.y", dev_y);
    node->get_parameter("fsm.deviation.z", dev_z);
    fsm_config.deviation = Eigen::Vector3d(dev_x, dev_y, dev_z);
    node->get_parameter("fsm.replan_interval", fsm_config.replan_interval_);
    node->get_parameter("fsm.replan_lateral_dev", fsm_config.replan_lateral_dev_);
    node->get_parameter("fsm.min_replan_interval", fsm_config.min_replan_interval_);
    node->get_parameter("fsm.goal_reached_radius", fsm_config.goal_reached_radius_);

    node->get_parameter("traj.rho_v", fsm_config.params_.rho_v);
    node->get_parameter("traj.rho_collision", fsm_config.params_.rho_collision);
    node->get_parameter("traj.rho_T", fsm_config.params_.rho_T);
    node->get_parameter("traj.rho_energy", fsm_config.params_.rho_energy);
    node->get_parameter("traj.max_v", fsm_config.params_.max_v);
    node->get_parameter("traj.rho_v_des", fsm_config.params_.rho_v_des);
    node->get_parameter("traj.v_des_ratio", fsm_config.params_.v_des_ratio);
    node->get_parameter("traj.safe_threshold",
                        fsm_config.params_.safe_threshold);
    node->get_parameter("traj.int_K", fsm_config.params_.int_K);
    node->get_parameter("traj.mem_size", fsm_config.params_.mem_size);
    node->get_parameter("traj.past", fsm_config.params_.past);
    node->get_parameter("traj.g_epsilon", fsm_config.params_.g_epsilon);
    node->get_parameter("traj.min_step", fsm_config.params_.min_step);
    node->get_parameter("traj.delta", fsm_config.params_.delta);
    node->get_parameter("traj.max_iter", fsm_config.params_.max_iter);

    // ========== 1. 声明 LMPC 参数 ==========
    node->declare_parameter<int>("lmpc.N", lmpc_param.N);
    node->declare_parameter<double>("lmpc.dt", lmpc_param.dt);

    // 控制输入边界 (u_min, u_max)
    node->declare_parameter<double>("lmpc.u_min_x", lmpc_param.u_min.x());
    node->declare_parameter<double>("lmpc.u_min_y", lmpc_param.u_min.y());
    node->declare_parameter<double>("lmpc.u_min_w", lmpc_param.u_min.z());
    node->declare_parameter<double>("lmpc.u_max_x", lmpc_param.u_max.x());
    node->declare_parameter<double>("lmpc.u_max_y", lmpc_param.u_max.y());
    node->declare_parameter<double>("lmpc.u_max_w", lmpc_param.u_max.z());

    // 状态边界 (x_min, x_max) - 通常用无穷大，但也可以设为地图边界
    node->declare_parameter<double>("lmpc.x_min_x", lmpc_param.x_min.x());
    node->declare_parameter<double>("lmpc.x_min_y", lmpc_param.x_min.y());
    node->declare_parameter<double>("lmpc.x_min_theta", lmpc_param.x_min.z());
    node->declare_parameter<double>("lmpc.x_max_x", lmpc_param.x_max.x());
    node->declare_parameter<double>("lmpc.x_max_y", lmpc_param.x_max.y());
    node->declare_parameter<double>("lmpc.x_max_theta", lmpc_param.x_max.z());

    // 权重矩阵 Q (对角元素)
    node->declare_parameter<double>("lmpc.Q_x", lmpc_param.Q.diagonal()(0));
    node->declare_parameter<double>("lmpc.Q_y", lmpc_param.Q.diagonal()(1));
    node->declare_parameter<double>("lmpc.Q_theta", lmpc_param.Q.diagonal()(2));

    // 权重矩阵 R (对角元素)
    node->declare_parameter<double>("lmpc.R_vx", lmpc_param.R.diagonal()(0));
    node->declare_parameter<double>("lmpc.R_vy", lmpc_param.R.diagonal()(1));
    node->declare_parameter<double>("lmpc.R_w", lmpc_param.R.diagonal()(2));

    // ========== 2. 读取 LMPC 参数 ==========
    node->get_parameter("lmpc.N", lmpc_param.N);
    node->get_parameter("lmpc.dt", lmpc_param.dt);

    double u_min_x, u_min_y, u_min_w;
    double u_max_x, u_max_y, u_max_w;
    node->get_parameter("lmpc.u_min_x", u_min_x);
    node->get_parameter("lmpc.u_min_y", u_min_y);
    node->get_parameter("lmpc.u_min_w", u_min_w);
    node->get_parameter("lmpc.u_max_x", u_max_x);
    node->get_parameter("lmpc.u_max_y", u_max_y);
    node->get_parameter("lmpc.u_max_w", u_max_w);
    lmpc_param.u_min = Eigen::Vector3d(u_min_x, u_min_y, u_min_w);
    lmpc_param.u_max = Eigen::Vector3d(u_max_x, u_max_y, u_max_w);

    double x_min_x, x_min_y, x_min_theta;
    double x_max_x, x_max_y, x_max_theta;
    node->get_parameter("lmpc.x_min_x", x_min_x);
    node->get_parameter("lmpc.x_min_y", x_min_y);
    node->get_parameter("lmpc.x_min_theta", x_min_theta);
    node->get_parameter("lmpc.x_max_x", x_max_x);
    node->get_parameter("lmpc.x_max_y", x_max_y);
    node->get_parameter("lmpc.x_max_theta", x_max_theta);
    lmpc_param.x_min = Eigen::Vector3d(x_min_x, x_min_y, x_min_theta);
    lmpc_param.x_max = Eigen::Vector3d(x_max_x, x_max_y, x_max_theta);

    double Q_x, Q_y, Q_theta;
    node->get_parameter("lmpc.Q_x", Q_x);
    node->get_parameter("lmpc.Q_y", Q_y);
    node->get_parameter("lmpc.Q_theta", Q_theta);
    lmpc_param.Q.diagonal() << Q_x, Q_y, Q_theta;

    double R_vx, R_vy, R_w;
    node->get_parameter("lmpc.R_vx", R_vx);
    node->get_parameter("lmpc.R_vy", R_vy);
    node->get_parameter("lmpc.R_w", R_w);
    lmpc_param.R.diagonal() << R_vx, R_vy, R_w;
  }
};
} // namespace planner