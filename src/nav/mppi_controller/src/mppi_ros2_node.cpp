#include <algorithm>
#include <atomic>
#include <geometry_msgs/msg/twist.hpp>
#include <memory>
#include <mutex>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/detail/point_cloud2__struct.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <thread>
#include <vector>

#include "../include/controller.hpp"

class MPPIRos2Node {
public:
  MPPIRos2Node(rclcpp::Node::SharedPtr &nh)
      : nh_(nh){
    initController();

    // 订阅者
    path_sub_ = nh_->create_subscription<nav_msgs::msg::Path>(
        "plan", 1,
        std::bind(&MPPIRos2Node::pathCallback, this, std::placeholders::_1));
    odom_sub_ = nh_->create_subscription<nav_msgs::msg::Odometry>(
        "odom", 10,
        std::bind(&MPPIRos2Node::odomCallback, this, std::placeholders::_1));
    scan_sub_ = nh_->create_subscription<sensor_msgs::msg::PointCloud2>(
        "scan", 10,
        std::bind(&MPPIRos2Node::scanCallback, this, std::placeholders::_1));

    // 发布者
    cmd_vel_pub_ =
        nh_->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    debug_traj_pub_ = nh_->create_publisher<nav_msgs::msg::Path>(
        "debug_optimal_trajectory", 10);

    // 定时器
    control_timer_ = nh_->create_wall_timer(
        std::chrono::duration<double>(control_period_ms_ / 1000.0),
        std::bind(&MPPIRos2Node::controlLoop, this));
    RCLCPP_INFO(nh->get_logger(), "MPPI ROS2 Node initialized.");
  }

private:
  void pathCallback(nav_msgs::msg::Path::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    if (msg->poses.empty())
      return;

    std::vector<mppi::Pose2D> path_poses;
    path_poses.reserve(msg->poses.size());
    for (const auto &p : msg->poses) {
      float yaw = std::atan2(
          2.0f * (p.pose.orientation.z * p.pose.orientation.w +
                  p.pose.orientation.x * p.pose.orientation.y),
          1.0f - 2.0f * (p.pose.orientation.y * p.pose.orientation.y +
                         p.pose.orientation.z * p.pose.orientation.z));
      path_poses.emplace_back(p.pose.position.x, p.pose.position.y, yaw);
    }
    controller_->setPath(path_poses);
    path_received_ = true;
    RCLCPP_INFO(nh_->get_logger(), "Path received with %zu points",
                msg->poses.size());
  }

  void odomCallback(nav_msgs::msg::Odometry::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    robot_pose_.x = msg->pose.pose.position.x;
    robot_pose_.y = msg->pose.pose.position.y;
    double yaw = std::atan2(
        2.0 * (msg->pose.pose.orientation.z * msg->pose.pose.orientation.w +
               msg->pose.pose.orientation.x * msg->pose.pose.orientation.y),
        1.0 -
            2.0 *
                (msg->pose.pose.orientation.y * msg->pose.pose.orientation.y +
                 msg->pose.pose.orientation.z * msg->pose.pose.orientation.z));
    robot_pose_.theta = static_cast<float>(yaw);
    robot_speed_.vx = msg->twist.twist.linear.x;
    robot_speed_.vy = msg->twist.twist.linear.y;
    robot_speed_.wz = msg->twist.twist.angular.z;
    pose_received_ = true;
  }

  void scanCallback(sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    laser_points_relative_.clear();
    laser_points_relative_.reserve(msg->width * msg->height);

    // 使用 PointCloud2 迭代器直接读取 x, y 字段
    sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");

    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y) {
      float x = *iter_x;
      float y = *iter_y;
      float dist = std::sqrt(x * x + y * y);
      if (dist < 4.0f) {
        laser_points_relative_.emplace_back(x, y);
      }
    }
  }

  void controlLoop() {
    if (!pose_received_ || !path_received_) {
      return;
    }

    mppi::Pose2D goal = controller_->getPath().getGoal();
    float dist_to_goal =
        std::hypot(robot_pose_.x - goal.x, robot_pose_.y - goal.y);
    if (dist_to_goal < 0.20) {
      geometry_msgs::msg::Twist stop_msg;
      cmd_vel_pub_->publish(stop_msg);
      return;
    }

    // 将相对激光点转换到全局坐标系
    std::vector<mppi::Point2D> global_points;
    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      global_points.reserve(laser_points_relative_.size());
      float cos_theta = std::cos(robot_pose_.theta);
      float sin_theta = std::sin(robot_pose_.theta);
      float robot_x = robot_pose_.x;
      float robot_y = robot_pose_.y;
      for (const auto &pt : laser_points_relative_) {
        float global_x = robot_x + pt.x * cos_theta - pt.y * sin_theta;
        float global_y = robot_y + pt.x * sin_theta + pt.y * cos_theta;
        global_points.emplace_back(global_x, global_y);
      }
    }

    // 更新障碍物
    controller_->updateStaticObstacles(global_points, robot_pose_);

    mppi::Twist2D cmd;
    try {
      cmd = controller_->computeVelocityCommands(robot_pose_, robot_speed_);
    } catch (const std::exception &e) {
      //   RCLCPP_WARN_THROTTLE(nh_->get_logger(), *nh_->get_clock(),
      //   std::chrono::milliseconds(1000), "MPPI compute failed: %s",
      //   e.what());
      RCLCPP_WARN(nh_->get_logger(), "MPPI compute failed");
      // 当所有轨迹都碰撞时，输出零速度，让机器人停在原地
      // 而不是抛出异常导致控制中断
      geometry_msgs::msg::Twist stop_msg;
      stop_msg.linear.x = 0.0;
      stop_msg.linear.y = 0.0;
      stop_msg.angular.z = 0.0;
      cmd_vel_pub_->publish(stop_msg);

      // 重置控制器状态，为下一次控制周期做准备
      controller_->reset();
      return;
    }

    if (tryApplyStartupAssist(cmd, global_points.empty(), dist_to_goal)) {
      RCLCPP_WARN(
          nh_->get_logger(),
          "MPPI: startup assist applied (near-zero cmd without obstacles).");
    }

    geometry_msgs::msg::Twist twist_msg;
    twist_msg.linear.x = cmd.vx;
    twist_msg.linear.y = cmd.vy;
    twist_msg.angular.z = cmd.wz;
    cmd_vel_pub_->publish(twist_msg);

    // 阻塞检测日志
    if (controller_->isCurrentlyBlocked()) {

      RCLCPP_WARN(nh_->get_logger(),
                  "MPPI: Blocked by obstacles, stopping robot.");
    }

    // 运动模型切换状态日志
    if (enable_omni_switching_) {
      static bool last_omni_state = false;
      bool current_omni_state = controller_->isOmniModeActive();
      if (current_omni_state != last_omni_state) {
        if (current_omni_state) {
          RCLCPP_WARN(
              nh_->get_logger(),
              "MPPI: Switched to OMNI mode (obstacle_dist=%.2f, path_dev=%.2f)",
              controller_->getMinObstacleDistance(),
              controller_->getCurrentPathDeviation());
        } else {
          RCLCPP_INFO(
              nh_->get_logger(),
              "MPPI: Restored DIFF mode (obstacle_dist=%.2f, path_dev=%.2f)",
              controller_->getMinObstacleDistance(),
              controller_->getCurrentPathDeviation());
        }
        last_omni_state = current_omni_state;
      }
    }

    publishDebugTrajectory();
  }

  bool tryApplyStartupAssist(mppi::Twist2D &cmd, bool no_obstacles,
                             float dist_to_goal) {
    // 如果启动辅助被禁用，直接返回false
    if (!startup_assist_enabled_)
      return false;

    if (!no_obstacles)
      return false;
    if (controller_->isCurrentlyBlocked())
      return false;
    if (dist_to_goal < startup_goal_distance_)
      return false;

    if (std::abs(cmd.vx) > startup_cmd_vx_eps_ ||
        std::abs(robot_speed_.vx) > startup_speed_vx_eps_) {
      return false;
    }

    auto &path = controller_->getPath();
    if (path.size() < 2)
      return false;

    size_t closest_idx = 0;
    float min_dist = std::numeric_limits<float>::max();
    for (size_t i = 0; i < path.size(); ++i) {
      float d =
          std::hypot(path.x(i) - robot_pose_.x, path.y(i) - robot_pose_.y);
      if (d < min_dist) {
        min_dist = d;
        closest_idx = i;
      }
    }

    size_t lookahead_idx = std::min(
        closest_idx + static_cast<size_t>(startup_lookahead_), path.size() - 1);
    float target_heading = std::atan2(path.y(lookahead_idx) - robot_pose_.y,
                                      path.x(lookahead_idx) - robot_pose_.x);
    float heading_error =
        mppi::shortestAngularDistance(robot_pose_.theta, target_heading);

    cmd.vx = startup_boost_vx_;
    cmd.vy = 0.0f;
    cmd.wz = std::clamp(startup_boost_wz_gain_ * heading_error,
                        -startup_boost_wz_limit_, startup_boost_wz_limit_);
    return true;
  }

  void publishDebugTrajectory() {
    auto optimal_traj = controller_->getOptimizedTrajectory();
    size_t T = optimal_traj.shape(0);
    nav_msgs::msg::Path path_msg;
    path_msg.header.stamp = nh_->now();
    path_msg.header.frame_id = "map";
    path_msg.poses.resize(T);
    for (size_t i = 0; i < T; ++i) {
      path_msg.poses[i].header = path_msg.header;
      path_msg.poses[i].pose.position.x = optimal_traj(i, 0);
      path_msg.poses[i].pose.position.y = optimal_traj(i, 1);
      float yaw = optimal_traj(i, 2);
      path_msg.poses[i].pose.orientation.z = std::sin(yaw / 2.0);
      path_msg.poses[i].pose.orientation.w = std::cos(yaw / 2.0);
    }
    debug_traj_pub_->publish(path_msg);
  }
  void initController() {
    mppi::OptimizerSettings settings;

    // MPPI 核心参数 —— 使用 int 获取后转换为 unsigned int
    int batch_size_tmp = nh_->declare_parameter("batch_size", 1300);
    int time_steps_tmp = nh_->declare_parameter("time_steps", 90);
    int iteration_count_tmp = nh_->declare_parameter("iteration_count", 1);
    int thread_count_tmp = nh_->declare_parameter("thread_count", 4);
    settings.batch_size = static_cast<unsigned int>(batch_size_tmp);
    settings.time_steps = static_cast<unsigned int>(time_steps_tmp);
    settings.iteration_count = static_cast<unsigned int>(iteration_count_tmp);
    settings.thread_count = static_cast<unsigned int>(thread_count_tmp);

    settings.model_dt = nh_->declare_parameter("model_dt", 0.05f);
    settings.temperature = nh_->declare_parameter("temperature", 0.3f);
    settings.gamma = nh_->declare_parameter("gamma", 0.015f);

    // 路径裁剪距离
    prune_distance_ = nh_->declare_parameter("prune_distance", 2.0f);
    settings.prune_distance = prune_distance_;

    // 控制约束
    settings.base_constraints.vx_max = nh_->declare_parameter("vx_max", 1.0f);
    settings.base_constraints.vx_min = nh_->declare_parameter("vx_min", 0.0f);
    settings.base_constraints.vy_max = nh_->declare_parameter("vy_max", 0.5f);
    settings.base_constraints.wz_max = nh_->declare_parameter("wz_max", 2.5f);
    settings.base_constraints.ax_max = nh_->declare_parameter("ax_max", 1.6f);
    settings.base_constraints.ay_max = nh_->declare_parameter("ay_max", 2.0f);
    settings.base_constraints.az_max = nh_->declare_parameter("az_max", 3.2f);
    settings.base_constraints.collision_cost_threshold =
        nh_->declare_parameter("collision_cost_threshold", 5000.0f);

    settings.constraints = settings.base_constraints;

    // 噪声采样标准差
    settings.sampling_std.vx = nh_->declare_parameter("vx_std", 0.30f);
    settings.sampling_std.vy = nh_->declare_parameter("vy_std", 0.0f);
    settings.sampling_std.wz = nh_->declare_parameter("wz_std", 0.30f);

    // 滤波器与控制策略
    settings.use_sg_filter = nh_->declare_parameter("use_sg_filter", false);
    settings.shift_control_sequence =
        nh_->declare_parameter("shift_control_sequence", false);
    settings.retry_attempt_limit =
        nh_->declare_parameter("retry_attempt_limit", 1);
    settings.use_mean_normalization =
        nh_->declare_parameter("use_mean_normalization", false);
    settings.adaptive_temperature =
        nh_->declare_parameter("adaptive_temperature", false);
    settings.adaptive_temperature_min =
        nh_->declare_parameter("adaptive_temperature_min", 0.1f);
    settings.adaptive_temperature_max =
        nh_->declare_parameter("adaptive_temperature_max", 1.0f);

    // 阻塞（打转）检测
    settings.twirling_weight = nh_->declare_parameter("twirling_weight", 0.5f);
    settings.spinning_ratio_threshold =
        nh_->declare_parameter("spinning_ratio_threshold", 8.0f);
    settings.spinning_detect_frames =
        nh_->declare_parameter("spinning_detect_frames", 4);

    // 全向/差速模式切换
    enable_omni_switching_ =
        nh_->declare_parameter("enable_omni_switching", false);
    settings.enable_omni_switching = enable_omni_switching_;
    settings.omni_trigger_obstacle_dist =
        nh_->declare_parameter("omni_trigger_obstacle_dist", 0.5f);
    settings.omni_trigger_path_deviation =
        nh_->declare_parameter("omni_trigger_path_deviation", 0.3f);
    settings.diff_restore_path_threshold =
        nh_->declare_parameter("diff_restore_path_threshold", 0.15f);
    settings.omni_switch_delay_frames =
        nh_->declare_parameter("omni_switch_delay_frames", 3);

    // 启动辅助
    startup_assist_enabled_ =
        nh_->declare_parameter("startup_assist_enabled", true);
    startup_boost_vx_ = nh_->declare_parameter("startup_boost_vx", 0.06f);
    startup_boost_wz_gain_ =
        nh_->declare_parameter("startup_boost_wz_gain", 1.2f);
    startup_boost_wz_limit_ =
        nh_->declare_parameter("startup_boost_wz_limit", 0.6f);
    startup_cmd_vx_eps_ = nh_->declare_parameter("startup_cmd_vx_eps", 0.01f);
    startup_speed_vx_eps_ =
        nh_->declare_parameter("startup_speed_vx_eps", 0.02f);
    startup_goal_distance_ =
        nh_->declare_parameter("startup_goal_distance", 0.30f);
    startup_lookahead_ =
        nh_->declare_parameter("startup_lookahead", 8); // int 类型

    // 运动模型
    std::string motion_model_str =
        nh_->declare_parameter("motion_model", std::string("DiffDrive"));
    double ackermann_radius =
        nh_->declare_parameter("ackermann_min_turning_radius", 0.2);

    controller_ = std::make_unique<mppi::MPPIController>();
    controller_->initialize(settings, motion_model_str, ackermann_radius);

    RCLCPP_INFO(nh_->get_logger(), "vx_min = %f",
                settings.base_constraints.vx_min);

    // ObstaclesCritic 参数
    if (auto *critic = controller_->getObstaclesCritic()) {
      double repulsion_weight =
          nh_->declare_parameter("obstacle_repulsion_weight", 0.5);
      double critical_weight =
          nh_->declare_parameter("obstacle_critical_weight", 20.0);
      double collision_cost =
          nh_->declare_parameter("obstacle_collision_cost", 10000.0);
      double collision_margin =
          nh_->declare_parameter("obstacle_collision_margin", 0.8);
      double inflation_radius =
          nh_->declare_parameter("obstacle_inflation_radius", 2.0);
      double cost_scaling =
          nh_->declare_parameter("obstacle_cost_scaling", 5.0);
      double near_goal_distance =
          nh_->declare_parameter("obstacle_near_goal_distance", 0.3);
      double robot_radius = nh_->declare_parameter("robot_radius", 0.25);
      double grid_resolution = nh_->declare_parameter("grid_resolution", 0.05);
      int grid_width = nh_->declare_parameter("grid_width", 100);
      int grid_height = nh_->declare_parameter("grid_height", 100);
      bool consider_footprint =
          nh_->declare_parameter("consider_footprint", false);

      std::vector<double> footprint_vec =
          nh_->declare_parameter("footprint", std::vector<double>());

      // 转换 footprint 为 Point2D 向量
      std::vector<mppi::Point2D> footprint;
      if (consider_footprint && !footprint_vec.empty()) {
        for (size_t i = 0; i + 1 < footprint_vec.size(); i += 2) {
          footprint.emplace_back(footprint_vec[i], footprint_vec[i + 1]);
        }
      }

      critic->setParams(repulsion_weight, collision_cost, collision_margin,
                        inflation_radius, cost_scaling, near_goal_distance,
                        robot_radius, grid_resolution, grid_width, grid_height,
                        consider_footprint, footprint);

      controller_->setRobotRadius(robot_radius);
    }

    // 路径对齐障碍物检查半径
    double path_align_check_radius =
        nh_->declare_parameter("path_align_obstacle_check_radius", 0.10);
    controller_->setPathAlignObstacleCheckRadius(path_align_check_radius);

    // Path Align Critic
    if (auto *critic = controller_->getPathAlignCritic()) {
      double weight = nh_->declare_parameter("path_align_weight", 6.0);
      int offset = nh_->declare_parameter("path_align_offset", 16);
      double threshold = nh_->declare_parameter("path_align_threshold", 0.40);
      int traj_step = nh_->declare_parameter("path_align_traj_step", 3);
      double max_occupancy_ratio =
          nh_->declare_parameter("path_align_max_occupancy_ratio", 0.50);
      bool use_orientations =
          nh_->declare_parameter("path_align_use_orientations", false);
      critic->setParams(weight, offset, threshold, traj_step,
                        max_occupancy_ratio, use_orientations);
    }

    // Path Angle Critic
    if (auto *critic = controller_->getPathAngleCritic()) {
      double weight = nh_->declare_parameter("path_angle_weight", 2.0);
      int offset = nh_->declare_parameter("path_angle_offset", 4);
      double threshold = nh_->declare_parameter("path_angle_threshold", 0.40);
      double angle_max = nh_->declare_parameter("path_angle_max", 0.5);
      int mode = nh_->declare_parameter("path_angle_mode", 0);
      critic->setParams(weight, offset, threshold, angle_max, mode);
    }

    // Path Follow Critic
    if (auto *critic = controller_->getPathFollowCritic()) {
      double weight = nh_->declare_parameter("path_follow_weight", 4.0);
      int offset = nh_->declare_parameter("path_follow_offset", 7);
      double threshold = nh_->declare_parameter("path_follow_threshold", 0.6);
      critic->setParams(weight, offset, threshold);
    }

    // Goal Critic
    if (auto *critic = controller_->getGoalCritic()) {
      double weight = nh_->declare_parameter("goal_weight", 5.0);
      double threshold = nh_->declare_parameter("goal_threshold", 1.0);
      critic->setParams(weight, threshold);
    }

    // Goal Angle Critic
    if (auto *critic = controller_->getGoalAngleCritic()) {
      double weight = nh_->declare_parameter("goal_angle_weight", 3.0);
      double threshold = nh_->declare_parameter("goal_angle_threshold", 0.4);
      critic->setParams(weight, threshold);
    }

    // Prefer Forward Critic
    if (auto *critic = controller_->getPreferForwardCritic()) {
      double weight = nh_->declare_parameter("prefer_forward_weight", 5.0);
      double threshold =
          nh_->declare_parameter("prefer_forward_threshold", 0.5);
      critic->setParams(weight, threshold);
    }

    // Constraint Critic
    if (auto *critic = controller_->getConstraintCritic()) {
      double weight = nh_->declare_parameter("constraint_weight", 4.0);
      double vx_max = nh_->declare_parameter("vx_max", 0.5);
      double vx_min = nh_->declare_parameter("vx_min", 0.0);
      double vy_max = nh_->declare_parameter("vy_max", 0.5);
      double wz_max = nh_->declare_parameter("wz_max", 1.9);
      double min_turning_radius =
          nh_->declare_parameter("ackermann_min_turning_radius", 0.2);
      int motion_model_type = nh_->declare_parameter("motion_model_type", 0);
      critic->setParams(weight, vx_max, vx_min, vy_max, wz_max,
                        min_turning_radius, motion_model_type);
    }

    // Velocity Deadband Critic
    if (auto *critic = controller_->getVelocityDeadbandCritic()) {
      double weight = nh_->declare_parameter("velocity_deadband_weight", 1.0);
      double vx = nh_->declare_parameter("velocity_deadband_vx", 0.05);
      double vy = nh_->declare_parameter("velocity_deadband_vy", 0.05);
      double wz = nh_->declare_parameter("velocity_deadband_wz", 0.1);
      critic->setParams(weight, vx, vy, wz);
    }

    // Twirling Critic
    if (auto *critic = controller_->getTwirlingCritic()) {
      double weight = nh_->declare_parameter("twirling_weight", 10.0);
      double vx_max = nh_->declare_parameter("vx_max", 0.5);
      double threshold = nh_->declare_parameter("twirling_threshold", 0.5);
      critic->setParams(weight, vx_max, threshold);
    }

    // 控制周期（int）
    control_period_ms_ = nh_->declare_parameter("control_period_ms", 50);
  }
  // Members
  rclcpp::Node::SharedPtr nh_;
  std::unique_ptr<mppi::MPPIController> controller_;
  mppi::Pose2D robot_pose_;
  mppi::Twist2D robot_speed_;
  std::vector<mppi::Point2D> laser_points_relative_;
  std::atomic<bool> pose_received_{false};
  std::atomic<bool> path_received_{false};
  int control_period_ms_;
  float prune_distance_ = 2.0f;        // 路径裁剪距离
  bool startup_assist_enabled_ = true; // 启动辅助开关
  float startup_boost_vx_ = 0.06f;
  float startup_boost_wz_gain_ = 1.2f;
  float startup_boost_wz_limit_ = 0.6f;
  float startup_cmd_vx_eps_ = 0.01f;
  float startup_speed_vx_eps_ = 0.02f;
  float startup_goal_distance_ = 0.30f;
  int startup_lookahead_ = 8;
  bool enable_omni_switching_ = false; // 全向/差速模式切换开关

  std::mutex data_mutex_;

  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr scan_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr debug_traj_pub_;
  rclcpp::TimerBase::SharedPtr control_timer_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("mppi_ros2_node");
    MPPIRos2Node mppi_node(node);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}