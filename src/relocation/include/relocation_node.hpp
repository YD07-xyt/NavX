#pragma once

#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "relocation.hpp"
#include "utils.hpp"
#include <chrono>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <memory>
#include <optional>
#include <pcl/impl/point_types.hpp>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/timer.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <spdlog/spdlog.h>
#include <string>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_broadcaster.hpp>
#include <tf2_ros/transform_listener.hpp>
namespace relocation {
class RelocationNode {
public:
  RelocationNode(const rclcpp::Node::SharedPtr node) : node_(node) {
    start_=std::chrono::steady_clock::now();
    accumulated_cloud_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(
        new pcl::PointCloud<pcl::PointXYZ>());
    relocation_user_ = std::make_shared<relocation::Relocation>();

    init_param();
    if (config_.default_init_pose.has_value()) {
      init_pose_ = config_.default_init_pose.value();
      spdlog::info("Using default initial pose from parameters.");
    }

    target_cloud_ = utils::read_pcd(config_.pcd_load_name);
    std::optional<pcl::PointCloud<pcl::PointXYZ>::Ptr> target_cloud_filter;
    if (target_cloud_.has_value()) {
      float voxel_filter_size = 0.25f;
      target_cloud_filter = utils::voxel_filter<pcl::PointXYZ>(
          target_cloud_.value(),
          Eigen::Vector3f{voxel_filter_size, voxel_filter_size,
                          voxel_filter_size});
      relocation_user_->init_target(*target_cloud_filter, "gicp");
      relocation_user_->init_target(*target_cloud_filter, "sac_ia");
      relocation_user_->init_target(*target_cloud_, "ndt");
    }

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node_->get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(node_);

    this->cloud_sub_ =
        node_->create_subscription<sensor_msgs::msg::PointCloud2>(
            config_.lio_cloud_topic_name, 10,
            [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
              this->PointCloudCallback(msg);
            });
    this->initial_pose_sub_ = node_->create_subscription<
        geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/initialpose", 10,
        [this](const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr
                   msg) { this->initialPoseCallback(msg); });

    this->map_cloud_pub_ =
        node_->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/relocation/map", 10);
    this->aligned_scan_pub_ =
        node_->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/relocation/aligned_scan", 10);
    this->main_timer_ =
        node_->create_wall_timer(std::chrono::milliseconds(100), // 2 Hz
                                 [this]() { MainCallback(); });

    this->transform_timer_ =
        node_->create_wall_timer(std::chrono::milliseconds(50), // 20 Hz
                                 [this]() { publishTransform(); });
    PubCloudMap(target_cloud_filter);
  }

private:
  std::shared_ptr<relocation::Relocation> relocation_user_;

private:
  std::optional<pcl::PointCloud<pcl::PointXYZ>::Ptr> target_cloud_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr scan_;

  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
      initial_pose_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_cloud_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr aligned_scan_pub_;
  rclcpp::TimerBase::SharedPtr main_timer_;
  rclcpp::TimerBase::SharedPtr transform_timer_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

private:
  std::chrono::steady_clock::time_point start_;
  bool relocation_is_success = false;
  rclcpp::Time last_scan_time_;
  std::string current_scan_frame_id_;
  std::optional<pcl::PointCloud<pcl::PointXYZ>::Ptr> accumulated_cloud_;
  std::optional<Eigen::Isometry3d> gicp_result_;
  std::optional<Eigen::Isometry3d> init_pose_;
  struct {
    std::string map_frame_ = "map";
    std::string odom_frame_ = "world";
    std::string robot_base_frame_ = "";
    std::string lio_cloud_topic_name = "";
    std::string pcd_load_name = "";
    float pcd_filter_size = 0.25;
    float accumulated_cloud_filter_size = 0.25;
    float min_sac_ia_score = 0.4;
    size_t min_accumulated_points_ = 8000; // 最少累积点数
    std::optional<Eigen::Isometry3d> default_init_pose;
    int max_relocation_time;
    int end_relocation_time=50;
    relocation::Relocation::RelocationConfig relocation_user_param;
  } config_;

  void PointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    last_scan_time_ = msg->header.stamp;
    current_scan_frame_id_ = msg->header.frame_id;

    pcl::PointCloud<pcl::PointXYZ>::Ptr scan(
        new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromROSMsg(*msg, *scan);
    **accumulated_cloud_ += *scan;
    // spdlog::info("receive pointcloud");
  };

  void MainCallback() {
    auto now = std::chrono::steady_clock::now();
    if (now - start_ > std::chrono::seconds(config_.max_relocation_time)&&relocation_is_success==true) {
      spdlog::info("now > max time: {},not relocation",config_.max_relocation_time);  
      return;
    }
    if (now - start_ > std::chrono::seconds(config_.end_relocation_time)) {
      spdlog::info("now >end time :{},not relocation",config_.end_relocation_time);  
      return;
    }
    if (!target_cloud_.has_value() || !accumulated_cloud_.has_value()) {
      return;
    }
    auto &cloud = accumulated_cloud_.value();
    if (cloud->empty())
      return;

    // 1. 核心优化：检查累积点数是否足够（太少的点无法做全局匹配）
    if (cloud->size() < config_.min_accumulated_points_) {
      spdlog::debug("Accumulating points: {}/{}", cloud->size(),
                   config_.min_accumulated_points_);
      return;
    }

    // 2. 预处理点云
    float voxel_filter_size = config_.accumulated_cloud_filter_size;
    auto accumulated_cloud_filter = utils::voxel_filter<pcl::PointXYZ>(
        cloud, Eigen::Vector3f{voxel_filter_size, voxel_filter_size,
                               voxel_filter_size});
    utils::remove_nan_points(accumulated_cloud_filter);

    spdlog::info("accumulated_cloud_filter : {} points ",
                 accumulated_cloud_filter->size());
    // auto sac_result=relocation_user_->sac_ia(accumulated_cloud_filter);
    // // 3. SAC-IA 粗匹配
    // if(!sac_result.has_value()){
    //   spdlog::warn("sac ia is null");
    //   return;
    // }
    // auto [score, tf_rough] =sac_result.value();
    // 初始位姿估计（示例）
    if (!init_pose_.has_value()) {
      spdlog::warn("No initial pose available, skipping NDT.");
      return;
    }
    Eigen::Matrix4f init_guess = init_pose_->matrix().cast<float>();

    auto [score, tf_rough] =
        relocation_user_->ndt(accumulated_cloud_filter, init_guess);
    spdlog::info("ndt: init_guess:{}", init_guess);
    if (score > config_.min_sac_ia_score) {
      spdlog::warn(
          "[SAC_IA] Score too high ({}), clearing and waiting for more points",
          score);
      cloud->clear();
      return;
    }

    Eigen::Matrix4d mat_d = tf_rough.cast<double>();
    std::optional<Eigen::Isometry3d> iso_tf_sac_ia(mat_d);

    // 4. GICP 精匹配
    auto gicp_result = relocation_user_->gicp(cloud, iso_tf_sac_ia);
    if (gicp_result.has_value() && gicp_result->matrix().allFinite()) {
      gicp_result_ = gicp_result;
      relocation_is_success = true;
      spdlog::info("[GICP] success converge");
    } else {
      // GICP 失败，除非 SAC-IA 本身非常出色，否则绝对不用 SAC-IA 的结果！
      // 此处我们选择不更新 gicp_result_，保持上次正确的位姿，直到下一次匹配成功
      spdlog::warn("[GICP] Failed to converge. Waiting for next accumulation.");
      gicp_result_=std::nullopt;
      // 将点云**留在 buffer 里**，不要
      // clear，让下一帧新点云加进来，增加匹配成功率
      return;
    }

    // 5. 发布对齐后的点云用于可视化
    if (relocation_is_success) {
      pcl::PointCloud<pcl::PointXYZ>::Ptr aligned_scan(
          new pcl::PointCloud<pcl::PointXYZ>);
      pcl::transformPointCloud(*cloud, *aligned_scan,
                               gicp_result_->matrix().cast<float>());
      sensor_msgs::msg::PointCloud2 aligned_msg;
      pcl::toROSMsg(*aligned_scan, aligned_msg);
      aligned_msg.header.frame_id = config_.map_frame_;
      aligned_msg.header.stamp = node_->now();
      aligned_scan_pub_->publish(aligned_msg);
    }

    // 6. 成功匹配后才清空点云，等待下一波数据
    cloud->clear();
  }

  void initialPoseCallback(
      const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
    spdlog::info("Received initial pose: [x: {:2f}, y: {:2f}, z: {:2f}]",
                 msg->pose.pose.position.x, msg->pose.pose.position.y,
                 msg->pose.pose.position.z);
    Eigen::Isometry3d map_to_robot_base = Eigen::Isometry3d::Identity();
    map_to_robot_base.translation() << msg->pose.pose.position.x,
        msg->pose.pose.position.y, msg->pose.pose.position.z;
    map_to_robot_base.linear() =
        Eigen::Quaterniond(
            msg->pose.pose.orientation.w, msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y, msg->pose.pose.orientation.z)
            .toRotationMatrix();

    try {
      auto transform = tf_buffer_->lookupTransform(config_.robot_base_frame_,
                                                   current_scan_frame_id_,
                                                   tf2::TimePointZero);

      // Eigen::Isometry3d robot_base_to_odom = Eigen::Isometry3d::Identity();
      // robot_base_to_odom.translation() << transform.transform.translation.x,
      //     transform.transform.translation.y,
      //     transform.transform.translation.z;
      // robot_base_to_odom.linear() =
      //     Eigen::Quaterniond(
      //         transform.transform.rotation.w, transform.transform.rotation.x,
      //         transform.transform.rotation.y, transform.transform.rotation.z)
      //         .toRotationMatrix();
      Eigen::Isometry3d robot_base_to_odom = tf2::transformToEigen(transform);

      Eigen::Isometry3d map_to_odom = map_to_robot_base * robot_base_to_odom;
      // map_to_odom.translation().z() = 0.0;
      init_pose_ = map_to_odom;
    } catch (tf2::TransformException &ex) {
      spdlog::warn("Could not transform initial pose from {} to {}: {}",
                   config_.robot_base_frame_.c_str(),
                   current_scan_frame_id_.c_str(), ex.what());
    }
  }
  void publishTransform() {

    geometry_msgs::msg::TransformStamped transform_stamped;
    // `+ 0.1` means transform into future. according to
    // https://robotics.stackexchange.com/a/96615
    transform_stamped.header.stamp =
        last_scan_time_ + rclcpp::Duration::from_seconds(0.1);
    transform_stamped.header.frame_id = config_.map_frame_;
    transform_stamped.child_frame_id = config_.odom_frame_;
    Eigen::Vector3d translation;
    Eigen::Quaterniond rotation;
    if (!gicp_result_.has_value()) {
      translation = init_pose_->translation();
      rotation = init_pose_->rotation();
    } else {
      translation = gicp_result_->translation();
      rotation = gicp_result_->rotation();
    }
    transform_stamped.transform.translation.x = translation.x();
    transform_stamped.transform.translation.y = translation.y();
    transform_stamped.transform.translation.z = translation.z();
    transform_stamped.transform.rotation.x = rotation.x();
    transform_stamped.transform.rotation.y = rotation.y();
    transform_stamped.transform.rotation.z = rotation.z();
    transform_stamped.transform.rotation.w = rotation.w();

    tf_broadcaster_->sendTransform(transform_stamped);
  }

  void PubCloudMap(std::optional<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloud) {
    // 检查地图点云是否已加载
    if (!cloud.has_value() || cloud.value()->empty()) {
      spdlog::warn("Target cloud is not loaded or empty, cannot publish map.");
      return;
    }

    sensor_msgs::msg::PointCloud2 map_msg;
    // 将 PCL 点云转换为 ROS 消息
    pcl::toROSMsg(*cloud.value(), map_msg);

    // 设置时间戳与坐标系
    map_msg.header.stamp = node_->now();
    map_msg.header.frame_id = config_.map_frame_; // "map"

    // 发布
    map_cloud_pub_->publish(map_msg);
    spdlog::info("Published map cloud with {} points.", cloud.value()->size());
  }
  void init_param() {
    // -------- 节点自身参数（地图坐标系、PCD路径等） --------
    node_->declare_parameter("map_frame", "map");
    node_->declare_parameter("odom_frame", "world");
    node_->declare_parameter("pcd_load_name", "");
    node_->declare_parameter("pcd_filter_size", 0.25);
    node_->declare_parameter("accumulated_cloud_filter_size", 0.25);
    node_->declare_parameter("min_sac_ia_score", 0.4);
    node_->declare_parameter("robot_base_frame", "imu");
    node_->declare_parameter("max_relocation_time", 40);
    node_->declare_parameter("end_relocation_time",300);
    node_->declare_parameter("lio_cloud_topic_name", "/lio/cloud_world");

    node_->get_parameter("map_frame", config_.map_frame_);
    node_->get_parameter("odom_frame", config_.odom_frame_);
    node_->get_parameter("pcd_load_name", config_.pcd_load_name);
    node_->get_parameter("pcd_filter_size", config_.pcd_filter_size);
    node_->get_parameter("accumulated_cloud_filter_size",
                         config_.accumulated_cloud_filter_size);
    node_->get_parameter("min_sac_ia_score", config_.min_sac_ia_score);
    node_->get_parameter("robot_base_frame", config_.robot_base_frame_);
    node_->get_parameter("max_relocation_time", config_.max_relocation_time);
    node_->get_parameter("end_relocation_time", config_.end_relocation_time);
    node_->get_parameter("lio_cloud_topic_name", config_.lio_cloud_topic_name);
    
    
    
    
    // -------- 加载默认初始位姿（新增） --------
    node_->declare_parameter("default_init_pose.x", 0.0);
    node_->declare_parameter("default_init_pose.y", 0.0);
    node_->declare_parameter("default_init_pose.z", 0.0);
    node_->declare_parameter("default_init_pose.qx", 0.0);
    node_->declare_parameter("default_init_pose.qy", 0.0);
    node_->declare_parameter("default_init_pose.qz", 0.0);
    node_->declare_parameter("default_init_pose.qw", 1.0);

    double x, y, z, qx, qy, qz, qw;
    node_->get_parameter("default_init_pose.x", x);
    node_->get_parameter("default_init_pose.y", y);
    node_->get_parameter("default_init_pose.z", z);
    node_->get_parameter("default_init_pose.qx", qx);
    node_->get_parameter("default_init_pose.qy", qy);
    node_->get_parameter("default_init_pose.qz", qz);
    node_->get_parameter("default_init_pose.qw", qw);

    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    pose.translation() << x, y, z;
    pose.linear() = Eigen::Quaterniond(qw, qx, qy, qz).toRotationMatrix();
    config_.default_init_pose = pose;
    spdlog::info("Loaded default initial pose: [x:{:.3f}, y:{:.3f}, z:{:.3f}]",
                 x, y, z);

    // -------- 加载 RelocationConfig 中所有参数 --------
    // GICP
    node_->declare_parameter("gicp_num_neighbors",
                             config_.relocation_user_param.gicp_num_neighbors_);
    node_->declare_parameter("gicp_num_threads",
                             config_.relocation_user_param.gicp_num_threads_);
    node_->declare_parameter("gicp_max_dist_sq",
                             config_.relocation_user_param.gicp_max_dist_sq_);
    node_->declare_parameter("gicp_max_iterations",
                             config_.relocation_user_param.gicp_max_iterations);
    node_->declare_parameter(
        "registered_leaf_size",
        config_.relocation_user_param.registered_leaf_size_);
    node_->declare_parameter("global_leaf_size",
                             config_.relocation_user_param.global_leaf_size_);

    node_->get_parameter("gicp_num_neighbors",
                         config_.relocation_user_param.gicp_num_neighbors_);
    node_->get_parameter("gicp_num_threads",
                         config_.relocation_user_param.gicp_num_threads_);
    node_->get_parameter("gicp_max_dist_sq",
                         config_.relocation_user_param.gicp_max_dist_sq_);
    node_->get_parameter("gicp_max_iterations",
                         config_.relocation_user_param.gicp_max_iterations);
    node_->get_parameter("registered_leaf_size",
                         config_.relocation_user_param.registered_leaf_size_);
    node_->get_parameter("global_leaf_size",
                         config_.relocation_user_param.global_leaf_size_);

    // FPFH
    node_->declare_parameter("fpfh_threads",
                             config_.relocation_user_param.fpfh_threads);
    node_->declare_parameter("fpfh_normal_radius",
                             config_.relocation_user_param.fpfh_normal_radius);
    node_->declare_parameter(
        "fpfh_estimator_radius",
        config_.relocation_user_param.fpfh_estimator_radius);

    node_->get_parameter("fpfh_threads",
                         config_.relocation_user_param.fpfh_threads);
    node_->get_parameter("fpfh_normal_radius",
                         config_.relocation_user_param.fpfh_normal_radius);
    node_->get_parameter("fpfh_estimator_radius",
                         config_.relocation_user_param.fpfh_estimator_radius);

    // SAC-IA
    node_->declare_parameter("sac_iterations",
                             config_.relocation_user_param.sac_iterations);
    node_->declare_parameter(
        "sac_min_sample_distance",
        config_.relocation_user_param.sac_min_sample_distance);
    node_->declare_parameter("sac_k_num",
                             config_.relocation_user_param.sac_k_num);

    node_->get_parameter("sac_iterations",
                         config_.relocation_user_param.sac_iterations);
    node_->get_parameter("sac_min_sample_distance",
                         config_.relocation_user_param.sac_min_sample_distance);
    node_->get_parameter("sac_k_num", config_.relocation_user_param.sac_k_num);

    // NDT（新增！）
    node_->declare_parameter(
        "ndt_voxel_filter_size",
        config_.relocation_user_param.ndt_voxel_filter_size);
    node_->declare_parameter("ndt_epsilon",
                             config_.relocation_user_param.ndt_epsilon);
    node_->declare_parameter("ndt_step_size",
                             config_.relocation_user_param.ndt_step_size);
    node_->declare_parameter("ndt_resolution",
                             config_.relocation_user_param.ndt_resolution);
    node_->declare_parameter(
        "ndt_maximum_iterations",
        config_.relocation_user_param.ndt_maximum_iterations);

    node_->get_parameter("ndt_voxel_filter_size",
                         config_.relocation_user_param.ndt_voxel_filter_size);
    node_->get_parameter("ndt_epsilon",
                         config_.relocation_user_param.ndt_epsilon);
    node_->get_parameter("ndt_step_size",
                         config_.relocation_user_param.ndt_step_size);
    node_->get_parameter("ndt_resolution",
                         config_.relocation_user_param.ndt_resolution);
    node_->get_parameter("ndt_maximum_iterations",
                         config_.relocation_user_param.ndt_maximum_iterations);

    // 将配置传递给 Relocation 类
    relocation_user_->init_param(config_.relocation_user_param);
    spdlog::info("[Node] init_param success");
  }
};
} // namespace relocation