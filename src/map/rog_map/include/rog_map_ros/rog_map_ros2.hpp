/**
 * This file is part of ROG-Map
 *
 * Copyright 2024 Yunfan REN, MaRS Lab, University of Hong Kong, <mars.hku.hk>
 * Developed by Yunfan REN <renyf at connect dot hku dot hk>
 * for more information see <https://github.com/hku-mars/ROG-Map>.
 * If you use this code, please cite the respective publications as
 * listed on the above website.
 *
 * ROG-Map is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * ROG-Map is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with ROG-Map. If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef ROG_MAP_ROS_HPP
#define ROG_MAP_ROS_HPP

#include "rog_map/esdf_map.h"
#include "super_utils/eigen_alias.hpp"
#include "terrain_analysis/terrain_analysis.hpp"
#include <memory>
#include <mutex>
#include <nav_msgs/msg/odometry.hpp>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/timer.hpp>
#include <sensor_msgs/msg/detail/point_cloud2__struct.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <unordered_set>
#include <visualization_msgs/msg/marker_array.hpp>

#include <rog_map/rog_map.h>
#include <std_srvs/srv/trigger.hpp>
#include <super_utils/color_msg_utils.hpp>

namespace rog_map {
using namespace super_utils;

class ROGMapROS : public ROGMap {

private:
  Terrain::TerrainAnalyzer terrain_analyzer_;

public:
  std::shared_ptr<ESDFMap> get_esdf_map() { return esdf_map_; };
  rclcpp::Node::SharedPtr nh_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> br_map_ego_;

  const double getSystemWalltimeNow() override {
    return nh_->get_clock()->now().seconds();
  }

  void getSystemWalltimeNow(rclcpp::Time &_in) {
    _in = nh_->get_clock()->now();
  };

  /// Accumulate occupied cells across sliding window for global map saving
  std::unordered_set<uint64_t> global_occ_keys_;
  std::mutex global_occ_mutex_;

  /// Encode global grid index into uint64_t key for set storage
  static inline uint64_t encodeGlobalKey(const Vec3i &id_g) {
    constexpr int64_t offset = 1LL << 20; // ±1M cells ≈ ±100km @ 0.1m
    int64_t x = static_cast<int64_t>(id_g.x()) + offset;
    int64_t y = static_cast<int64_t>(id_g.y()) + offset;
    int64_t z = static_cast<int64_t>(id_g.z()) + offset;
    return (static_cast<uint64_t>(x) << 42) | (static_cast<uint64_t>(y) << 21) |
           static_cast<uint64_t>(z);
  }

  /// Decode key back to global grid index
  static inline Vec3i decodeGlobalKey(uint64_t key) {
    constexpr int64_t offset = 1LL << 20;
    int64_t z = static_cast<int64_t>(key & ((1ULL << 21) - 1)) - offset;
    int64_t y = static_cast<int64_t>((key >> 21) & ((1ULL << 21) - 1)) - offset;
    int64_t x = static_cast<int64_t>((key >> 42) & ((1ULL << 21) - 1)) - offset;
    return Vec3i(static_cast<int>(x), static_cast<int>(y), static_cast<int>(z));
  }

  struct VisualizeMap {
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr occ_pub,
        unknown_pub, esdf_neg_pub, esdf_occ_pub, occ_inf_pub, unknown_inf_pub,
        frontier_pub, esdf_pub, terrain_map_pub, global_3docc_pub,
        global_terrain_pub;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
        mkr_arr_pub;
    rclcpp::TimerBase::SharedPtr viz_timer;
    rclcpp::CallbackGroup::SharedPtr viz_reen_cbk_group;
    PointCloud::Ptr global_pcd_map_;
    PointCloud::Ptr global_terrain_map_;
    sensor_msgs::msg::PointCloud2::SharedPtr global_3docc_msg_;
    sensor_msgs::msg::PointCloud2::SharedPtr global_terrain_msg_;
  } vm_;

  struct ROSCallback {
    rclcpp::CallbackGroup::SharedPtr odom_me_cbk_group, cloud_me_cbk_group,
        update_cbk_group, terrain_callback_group_, global_map_callback_group_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub;
    int unfinished_frame_cnt{0};
    Pose pc_pose;
    PointCloud pc;
    rclcpp::TimerBase::SharedPtr update_timer;
    rclcpp::TimerBase::SharedPtr terrain_timer_;
    rclcpp::TimerBase::SharedPtr global_map_timer_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr save_map_srv_;

    mutex updete_lock;
  } rc_;
  void terrainCallback() {
    // Vec3f robot_pos = robot_state_.p;
    Vec3f box_max = robot_state_.p + cfg_.visualization_range / 2;
    Vec3f box_min = robot_state_.p - cfg_.visualization_range / 2;
    rog_map::vec_E<Vec3f> inf_occ_map, real_occ_map;
    boxSearchInflate(box_min, box_max, OCCUPIED, inf_occ_map);
    boxSearch(box_min, box_max, OCCUPIED, real_occ_map);
    auto terrain_map = terrain_analyzer_.analyze(robot_state_, real_occ_map);
    // vec_E<Vec3f> output;
    // for(auto pt:terrain_map){
    //   if(pt.z()-robot_state_.p.z()<0.07){
    //    //continue;
    //   }
    //   output.emplace_back(pt);
    // }
    sensor_msgs::msg::PointCloud2 cloud_msg;
    vecEVec3fToPC2(terrain_map, cloud_msg);
    cloud_msg.header.stamp = nh_->get_clock()->now();
    vm_.terrain_map_pub->publish(cloud_msg);
  }

  void globalMapCallback(
      const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
    (void)request;
    if (!cfg_.global_map_en) {
      response->success = false;
      response->message = "Global map feature is not enabled in config.";
      return;
    }
    publishGlobalMap();
    response->success = true;
    response->message = "Global map published successfully.";
  }

  void publishGlobalMap() {
    if (!vm_.global_pcd_map_) {
      return;
    }

    rclcpp::Time now = nh_->get_clock()->now();

    if (vm_.global_3docc_pub) {
      vm_.global_3docc_msg_->header.stamp = now;
      vm_.global_3docc_pub->publish(*vm_.global_3docc_msg_);
    }

    if (vm_.global_terrain_pub) {
      vm_.global_terrain_msg_->header.stamp = now;
      vm_.global_terrain_pub->publish(*vm_.global_terrain_msg_);
    }
  }

  // ============== Save Map to Disk ==============

  void saveMapCallback(
      const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
    (void)request;
    if (!cfg_.save_map_en) {
      response->success = false;
      response->message = "save_map is not enabled in config.";
      return;
    }
    response->success = saveMap();
    response->message = response->success ? "Map saved successfully."
                                          : "Failed to save map (see log).";
  }

  bool saveMap() {
    // 1) create save directory
    const string dir = cfg_.save_map_dir;
    const string cmd = string("mkdir -p ") + dir;
    if (system(cmd.c_str()) != 0) {
      RCLCPP_ERROR(nh_->get_logger(), "Failed to create dir: %s", dir.c_str());
      return false;
    }

    // 2) also merge current sliding window into the global accumulator
    //    (in case vizCallback hasn't run recently)
    {
      Vec3f cur_min = local_map_bound_min_d_;
      Vec3f cur_max = local_map_bound_max_d_;
      cur_min.z() = std::max(cur_min.z(), cfg_.virtual_ground_height);
      cur_max.z() = std::min(cur_max.z(), cfg_.virtual_ceil_height);
      vec_E<Vec3f> cur_occ;
      boxSearch(cur_min, cur_max, OCCUPIED, cur_occ);
      std::lock_guard<std::mutex> lock(global_occ_mutex_);
      Vec3i id_g;
      for (const auto &pt : cur_occ) {
        posToGlobalIndex(pt, id_g);
        global_occ_keys_.insert(encodeGlobalKey(id_g));
      }
    }

    // 3) convert accumulated global set → point cloud + bounding box
    vec_E<Vec3f> occ_points;
    {
      std::lock_guard<std::mutex> lock(global_occ_mutex_);
      occ_points.reserve(global_occ_keys_.size());
      for (const auto &key : global_occ_keys_) {
        Vec3i id_g = decodeGlobalKey(key);
        Vec3f pos;
        globalIndexToPos(id_g, pos);
        occ_points.emplace_back(pos);
      }
    }
    RCLCPP_INFO(nh_->get_logger(), "Global accumulated occupied cells: %zu",
                occ_points.size());

    if (occ_points.empty()) {
      RCLCPP_WARN(nh_->get_logger(), "No occupied cells accumulated.");
      return false;
    }

    // 4) compute global bounding box from accumulated points
    Vec3f box_min(1e9, 1e9, 1e9), box_max(-1e9, -1e9, -1e9);
    for (const auto &pt : occ_points) {
      box_min = box_min.cwiseMin(pt);
      box_max = box_max.cwiseMax(pt);
    }
    // Add a small margin
    box_min -= Vec3f(cfg_.resolution, cfg_.resolution, 0);
    box_max += Vec3f(cfg_.resolution, cfg_.resolution, 0);
    box_min.z() = std::max(box_min.z(), cfg_.virtual_ground_height);
    box_max.z() = std::min(box_max.z(), cfg_.virtual_ceil_height);

    // 5) voxel-downsample for manageable PCD size (0.2m leaf)
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_3d(
        new pcl::PointCloud<pcl::PointXYZ>);
    cloud_3d->resize(occ_points.size());
    for (size_t i = 0; i < occ_points.size(); ++i) {
      (*cloud_3d)[i].x = occ_points[i].x();
      (*cloud_3d)[i].y = occ_points[i].y();
      (*cloud_3d)[i].z = occ_points[i].z();
    }
    pcl::VoxelGrid<pcl::PointXYZ> voxel;
    voxel.setInputCloud(cloud_3d);
    voxel.setLeafSize(0.2f, 0.2f, 0.2f);
    pcl::PointCloud<pcl::PointXYZ> cloud_ds;
    voxel.filter(cloud_ds);

    // convert back to vec_E for terrain analysis
    vec_E<Vec3f> occ_ds;
    occ_ds.reserve(cloud_ds.size());
    for (const auto &pt : cloud_ds) {
      occ_ds.emplace_back(pt.x, pt.y, pt.z);
    }

    // 6) save 3D occ PCD (full accumulated)
    if (cfg_.save_3docc_pcd_en) {
      const string occ_pcd = dir + "/3d_occ.pcd";
      if (pcl::io::savePCDFileBinary(occ_pcd, cloud_ds) == 0) {
        RCLCPP_INFO(nh_->get_logger(), "Saved global 3D occ map: %s (%zu pts)",
                    occ_pcd.c_str(), cloud_ds.size());
      } else {
        RCLCPP_ERROR(nh_->get_logger(), "Failed to save: %s", occ_pcd.c_str());
      }
    }

    // 7) terrain analysis on global accumulated data
    RobotState fake_pose;
    fake_pose.p = (box_min + box_max) * 0.5f;
    fake_pose.q = Quatf::Identity();
    fake_pose.rcv = true;
    fake_pose.yaw = 0.0;

    Terrain::TerrainAnalyzer::Config terrain_cfg;
    terrain_cfg.resolution = cfg_.resolution;
    terrain_cfg.map_size_x = (box_max.x() - box_min.x()) + cfg_.resolution * 2;
    terrain_cfg.map_size_y = (box_max.y() - box_min.y()) + cfg_.resolution * 2;
    terrain_cfg.kernel_size = cfg_.terrain_kernel_size;
    terrain_cfg.max_step_height = cfg_.terrain_max_step_height;
    terrain_cfg.robot_height = cfg_.terrain_robot_height;
    terrain_cfg.steep_threshold = cfg_.terrain_steep_threshold;
    Terrain::TerrainAnalyzer global_terrain_analyzer(terrain_cfg);

    auto terrain_pts = global_terrain_analyzer.analyze(fake_pose, occ_ds);
    RCLCPP_INFO(nh_->get_logger(), "Global terrain obstacle points: %zu",
                terrain_pts.size());

    // 8) save terrain PCD
    if (cfg_.save_terrain_pcd_en) {
      pcl::PointCloud<pcl::PointXYZ> cloud_t;
      cloud_t.resize(terrain_pts.size());
      for (size_t i = 0; i < terrain_pts.size(); ++i) {
        cloud_t[i].x = terrain_pts[i].x();
        cloud_t[i].y = terrain_pts[i].y();
        cloud_t[i].z = terrain_pts[i].z();
      }
      const string t_pcd = dir + "/terrain_obstacle.pcd";
      if (pcl::io::savePCDFileBinary(t_pcd, cloud_t) == 0) {
        RCLCPP_INFO(nh_->get_logger(), "Saved global terrain map: %s (%zu pts)",
                    t_pcd.c_str(), cloud_t.size());
      } else {
        RCLCPP_ERROR(nh_->get_logger(), "Failed to save: %s", t_pcd.c_str());
      }
    }

    // 9) save terrain PGM (global bounds)
    if (cfg_.save_terrain_pgm_en) {
      saveTerrainPGM(dir + "/terrain_costmap.pgm", terrain_pts, box_min,
                     box_max);
    }

    RCLCPP_INFO(nh_->get_logger(),
                "Global map saved. Bounding box: [%.1f,%.1f] × [%.1f,%.1f]",
                box_min.x(), box_max.x(), box_min.y(), box_max.y());
    return true;
  }

  void saveTerrainPGM(const string &pgm_path, const vec_E<Vec3f> &terrain_pts,
                      const Vec3f &box_min, const Vec3f &box_max) {
    const double res = cfg_.resolution;
    const int w =
        static_cast<int>(std::ceil((box_max.x() - box_min.x()) / res));
    const int h =
        static_cast<int>(std::ceil((box_max.y() - box_min.y()) / res));
    if (w <= 0 || h <= 0) {
      RCLCPP_ERROR(nh_->get_logger(), "PGM dimensions invalid: %dx%d", w, h);
      return;
    }

    // build 2D obstacle grid (0 = free, 255 = obstacle)
    std::vector<uint8_t> grid(w * h, 0);

    // rasterize terrain obstacle points onto the grid
    for (const auto &pt : terrain_pts) {
      int col = static_cast<int>((pt.x() - box_min.x()) / res);
      int row = static_cast<int>((pt.y() - box_min.y()) / res);
      col = std::max(0, std::min(w - 1, col));
      row = std::max(0, std::min(h - 1, row));
      grid[row * w + col] = 255; // obstacle
    }

    // write PGM (P5 binary)
    std::ofstream f(pgm_path, std::ios::binary);
    if (!f.is_open()) {
      RCLCPP_ERROR(nh_->get_logger(), "Cannot open %s", pgm_path.c_str());
      return;
    }
    // header
    f << "P5\n# ROG-Map terrain costmap\n" << w << " " << h << "\n255\n";
    f.write(reinterpret_cast<const char *>(grid.data()),
            static_cast<std::streamsize>(grid.size()));
    f.close();
    RCLCPP_INFO(nh_->get_logger(), "Saved terrain PGM: %s (%dx%d)",
                pgm_path.c_str(), w, h);
  }

  void loadGlobalPCD() {
    if (!cfg_.global_map_en || cfg_.global_map_pcd_path.empty()) {
      return;
    }

    PointCloud::Ptr pcd_map(new PointCloud);
    if (pcl::io::loadPCDFile(cfg_.global_map_pcd_path, *pcd_map) == -1) {
      RCLCPP_ERROR(nh_->get_logger(), "Load pcd file at [%s] failed!",
                   cfg_.global_map_pcd_path.c_str());
      return;
    }
    // 1. 统计离群点滤波（剔除环境中的离散噪声）
    PointCloud::Ptr pcd_filtered_noise(new PointCloud);
    pcl::StatisticalOutlierRemoval<PointCloud::PointType> sor1;
    sor1.setInputCloud(pcd_map);
    sor1.setMeanK(30);            // 邻域点数量，越大越严格
    sor1.setStddevMulThresh(1.0); // 标准差倍数阈值
    sor1.filter(*pcd_filtered_noise);

    PointCloud::Ptr pcd_downsampled(
        new PointCloud); // 这里 PointCloud 自动对应原类型
    pcl::VoxelGrid<PointCloud::PointType>
        sor; // 使用 PointCloud::PointType 获取点类型
    sor.setInputCloud(pcd_filtered_noise);
    sor.setLeafSize(0.2f, 0.2f, 0.2f);
    sor.filter(*pcd_downsampled);

    RCLCPP_INFO(nh_->get_logger(), "Loaded global PCD with %lu pts.",
                pcd_map->size());
    RCLCPP_INFO(nh_->get_logger(), "Loaded global PCD with %lu pts.",
                pcd_downsampled->size());
    Vec3f bbox_min(1e9, 1e9, 1e9), bbox_max(-1e9, -1e9, -1e9);
    for (const auto &pt : *pcd_downsampled) {
      bbox_min.x() = std::min(bbox_min.x(), static_cast<double>(pt.x));
      bbox_min.y() = std::min(bbox_min.y(), static_cast<double>(pt.y));
      bbox_min.z() = std::min(bbox_min.z(), static_cast<double>(pt.z));
      bbox_max.x() = std::max(bbox_max.x(), static_cast<double>(pt.x));
      bbox_max.y() = std::max(bbox_max.y(), static_cast<double>(pt.y));
      bbox_max.z() = std::max(bbox_max.z(), static_cast<double>(pt.z));
    }
    Vec3f center = (bbox_min + bbox_max) / 2.0f;
    Vec3f terrain_origin = cfg_.global_map_terrain_origin;
    if (terrain_origin.x() != 0.0f || terrain_origin.y() != 0.0f ||
        terrain_origin.z() != 0.0f) {
      center = terrain_origin;
    }

    RobotState fake_robot;
    fake_robot.p = center;
    fake_robot.q = Quatf::Identity();
    fake_robot.rcv = true;
    fake_robot.yaw = 0.0;

    Terrain::TerrainAnalyzer::Config terrain_cfg;
    terrain_cfg.resolution = cfg_.resolution;
    terrain_cfg.map_size_x =
        (bbox_max.x() - bbox_min.x()) + cfg_.resolution * 2;
    terrain_cfg.map_size_y =
        (bbox_max.y() - bbox_min.y()) + cfg_.resolution * 2;
    terrain_cfg.kernel_size = cfg_.terrain_kernel_size;
    terrain_cfg.max_step_height = cfg_.terrain_max_step_height;
    terrain_cfg.robot_height = cfg_.terrain_robot_height;
    terrain_cfg.steep_threshold = cfg_.terrain_steep_threshold;
    Terrain::TerrainAnalyzer global_terrain_analyzer(terrain_cfg);

    rog_map::vec_E<Vec3f> pcd_points;
    pcd_points.reserve(pcd_downsampled->size());
    for (const auto &pt : *pcd_downsampled) {
      pcd_points.emplace_back(pt.x, pt.y, pt.z);
    }

    auto terrain_map = global_terrain_analyzer.analyze(fake_robot, pcd_points);

    PointCloud::Ptr terrain_pcd(new PointCloud);
    terrain_pcd->resize(terrain_map.size());
    for (size_t i = 0; i < terrain_map.size(); ++i) {
      (*terrain_pcd)[i].x = terrain_map[i].x();
      (*terrain_pcd)[i].y = terrain_map[i].y();
      (*terrain_pcd)[i].z = terrain_map[i].z();
    }

    vm_.global_pcd_map_ = pcd_downsampled;
    vm_.global_terrain_map_ = terrain_pcd;

    vm_.global_3docc_msg_ = std::make_shared<sensor_msgs::msg::PointCloud2>();
    pcl::toROSMsg(*pcd_downsampled, *vm_.global_3docc_msg_);
    vm_.global_3docc_msg_->header.frame_id = "world";

    vm_.global_terrain_msg_ = std::make_shared<sensor_msgs::msg::PointCloud2>();
    pcl::toROSMsg(*terrain_pcd, *vm_.global_terrain_msg_);
    vm_.global_terrain_msg_->header.frame_id = "world";

    RCLCPP_INFO(nh_->get_logger(), "Global terrain_map prepared with %lu pts.",
                terrain_map.size());
  }
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr odom_msg) {
    updateRobotState(std::make_pair(Vec3f(odom_msg->pose.pose.position.x,
                                          odom_msg->pose.pose.position.y,
                                          odom_msg->pose.pose.position.z),
                                    Quatf(odom_msg->pose.pose.orientation.w,
                                          odom_msg->pose.pose.orientation.x,
                                          odom_msg->pose.pose.orientation.y,
                                          odom_msg->pose.pose.orientation.z)));

    geometry_msgs::msg::TransformStamped transformStamped;
    transformStamped.header.stamp = nh_->get_clock()->now();
    transformStamped.header.frame_id = "world";
    transformStamped.child_frame_id = "drone";
    transformStamped.transform.translation.x = odom_msg->pose.pose.position.x;
    transformStamped.transform.translation.y = odom_msg->pose.pose.position.y;
    transformStamped.transform.translation.z = odom_msg->pose.pose.position.z;
    transformStamped.transform.rotation.x = odom_msg->pose.pose.orientation.x;
    transformStamped.transform.rotation.y = odom_msg->pose.pose.orientation.y;
    transformStamped.transform.rotation.z = odom_msg->pose.pose.orientation.z;
    transformStamped.transform.rotation.w = odom_msg->pose.pose.orientation.w;
    br_map_ego_->sendTransform(transformStamped);
  }

  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg) {
    if (!robot_state_.rcv) {
      std::cout << YELLOW << " -- [ROS] No odom received, skip cloud callback."
                << RESET << std::endl;
      return;
    }
    double cbk_t = nh_->get_clock()->now().seconds();
    if (cbk_t - robot_state_.rcv_time > cfg_.odom_timeout) {
      std::cout << YELLOW << " -- [ROS] Odom timeout, skip cloud callback."
                << RESET << std::endl;
      return;
    }

    PointCloud temp_pc;
    pcl::fromROSMsg(*cloud_msg, temp_pc);
    rc_.updete_lock.lock();
    rc_.pc = temp_pc;
    rc_.pc_pose = std::make_pair(robot_state_.p, robot_state_.q);
    rc_.unfinished_frame_cnt++;
    map_empty_ = false;
    rc_.updete_lock.unlock();
  }

  void updateCallback() {
    if (map_empty_) {
      static double last_print_t = nh_->get_clock()->now().seconds();
      double cur_t = nh_->get_clock()->now().seconds();
      if (cfg_.ros_callback_en && (cur_t - last_print_t > 1.0)) {
        std::cout
            << YELLOW
            << " -- [ROG WARN] No point cloud input, check the topic name."
            << RESET << std::endl;
        last_print_t = cur_t;
      }
      return;
    }
    if (rc_.unfinished_frame_cnt == 0) {
      return;
    }

    if (rc_.unfinished_frame_cnt > 1) {
      std::cout << YELLOW
                << " -- [ROG WARN] Unfinished frame cnt > 1, the map may not "
                   "work in real-time"
                << RESET << std::endl;
    }
    static PointCloud temp_pc;
    static Pose temp_pose;

    rc_.updete_lock.lock();
    temp_pc = rc_.pc;
    temp_pose = rc_.pc_pose;
    rc_.unfinished_frame_cnt = 0;
    rc_.updete_lock.unlock();

    updateProbMap(temp_pc, temp_pose);

    writeTimeConsumingToLog(time_log_file_);
  }

  void vizCallback() {
    if (!cfg_.visualization_en) {
      return;
    }
    if (map_empty_) {
      return;
    }

    Vec3f box_max = robot_state_.p + cfg_.visualization_range / 2;
    Vec3f box_min = robot_state_.p - cfg_.visualization_range / 2;

    boundBoxByLocalMap(box_min, box_max);
    if ((box_max - box_min).minCoeff() <= 0) {
      cout << YELLOW << " -- [ROGMap] Visualization range is too small."
           << RESET << endl;
      return;
    }

    if (cfg_.pub_unknown_map_en &&
        vm_.unknown_pub->get_subscription_count() >= 1) {
      vec_E<Vec3f> unknown_map, inf_unknown_map;
      boxSearch(box_min, box_max, UNKNOWN, unknown_map);
      sensor_msgs::msg::PointCloud2 cloud_msg;
      vecEVec3fToPC2(unknown_map, cloud_msg);
      cloud_msg.header.stamp = nh_->get_clock()->now();
      vm_.unknown_pub->publish(cloud_msg);
      if (cfg_.unk_inflation_en &&
          vm_.unknown_inf_pub->get_subscription_count() >= 1) {
        boxSearchInflate(box_min, box_max, UNKNOWN, inf_unknown_map);
        vecEVec3fToPC2(inf_unknown_map, cloud_msg);
        cloud_msg.header.stamp = nh_->get_clock()->now();
        vm_.unknown_inf_pub->publish(cloud_msg);
      }
    }

    if (cfg_.frontier_extraction_en &&
        vm_.frontier_pub->get_subscription_count() >= 1) {
      vec_E<Vec3f> frontier_map;
      boxSearch(box_min, box_max, FRONTIER, frontier_map);
      sensor_msgs::msg::PointCloud2 cloud_msg;
      vecEVec3fToPC2(frontier_map, cloud_msg);
      cloud_msg.header.stamp = nh_->get_clock()->now();
      vm_.frontier_pub->publish(cloud_msg);
    }

    vec_E<Vec3f> occ_map, inf_occ_map;
    sensor_msgs::msg::PointCloud2 cloud_msg;

    if (vm_.occ_pub->get_subscription_count() >= 1) {
      boxSearch(box_min, box_max, OCCUPIED, occ_map);
      vecEVec3fToPC2(occ_map, cloud_msg);
      vm_.occ_pub->publish(cloud_msg);

      // Accumulate occupied cells for global map saving (across sliding
      // windows)
      if (cfg_.save_map_en) {
        std::lock_guard<std::mutex> lock(global_occ_mutex_);
        Vec3i id_g;
        for (const auto &pt : occ_map) {
          posToGlobalIndex(pt, id_g);
          global_occ_keys_.insert(encodeGlobalKey(id_g));
        }
      }
    }

    if (vm_.occ_inf_pub->get_subscription_count() >= 1) {
      boxSearchInflate(box_min, box_max, OCCUPIED, inf_occ_map);
      vecEVec3fToPC2(inf_occ_map, cloud_msg);
      cloud_msg.header.stamp = nh_->get_clock()->now();
      vm_.occ_inf_pub->publish(cloud_msg);
    }

    /* visualize ESDF Map*/
    if (cfg_.esdf_en) {
      if (vm_.esdf_pub->get_subscription_count() >= 1) {
        PointCloud pc;
        esdf_map_->getPositiveESDFPointCloud(box_min, box_max,
                                             robot_state_.p.z() - 0.5, pc);
        pcl::toROSMsg(pc, cloud_msg);
        cloud_msg.header.frame_id = "world";
        cloud_msg.header.stamp = nh_->get_clock()->now();
        vm_.esdf_pub->publish(cloud_msg);
      }

      // if (vm_.esdf_neg_pub->get_subscription_count() >= 1) {
      //     PointCloud pc;
      //     esdf_map_->getNegativeESDFPointCloud(box_min, box_max,
      //     robot_state_.p.z() - 0.5, pc); pcl::toROSMsg(pc, cloud_msg);
      //     cloud_msg.header.frame_id = "world";
      //     cloud_msg.header.stamp = nh_->get_clock()->now();
      //     vm_.esdf_neg_pub->publish(cloud_msg);
      // }

#ifdef ESDF_MAP_DEBUG
      esdf_map_->getESDFOccPC2(box_min, box_max, cloud_msg);
      cloud_msg.header.stamp = nh_->get_clock()->now();
      vm_.esdf_occ_pub->publish(cloud_msg);
#endif
    }

    /* Publish visualization range */
    visualization_msgs::msg::MarkerArray mkr_arr;
    visualizeBoundingBox(mkr_arr, nh_->get_clock()->now().seconds(), box_min,
                         box_max, "Visualization Range", Color::Purple());
    visualizeText(mkr_arr, nh_->get_clock()->now().seconds(),
                  "Visualization Range Text", "Visualization Range",
                  box_max + Vec3f(0, 0, 0.5), Color::Purple(), 0.6, 0);

    /* Publish local map range */
    Vec3f local_map_max(999, 999, 999), local_map_min(-999, -999, -999);
    boundBoxByLocalMap(local_map_min, local_map_max);
    visualizeBoundingBox(mkr_arr, nh_->get_clock()->now().seconds(),
                         local_map_min, local_map_max, "Local Map Range",
                         Color::Orange());
    visualizeText(mkr_arr, nh_->get_clock()->now().seconds(),
                  "Local Map Range Text", "Local Map Range",
                  local_map_max + Vec3f(0, 0, 1.0), Color::Orange(), 0.6, 0);

    /* Publish Ray-casting range */
    visualizeBoundingBox(
        mkr_arr, nh_->get_clock()->now().seconds(), raycast_data_.cache_box_min,
        raycast_data_.cache_box_max, "Updating Range", Color::Green());
    visualizeText(mkr_arr, nh_->get_clock()->now().seconds(),
                  "Updating Range Text", "Updating Range",
                  raycast_data_.cache_box_max + Vec3f(0, 0, 0.5),
                  Color::Green(), 0.6, 0);

    /* Publish Local map origin */
    visualizePoint(mkr_arr, nh_->get_clock()->now().seconds(),
                   local_map_origin_d_, Color::Red(), "Local Map Origin", 0.2,
                   0);

    if (cfg_.esdf_en) {
      Vec3f esdf_box_max, esdf_box_min;
      esdf_map_->getUpdatedBbox(esdf_box_min, esdf_box_max);
      visualizeText(mkr_arr, nh_->get_clock()->now().seconds(), "ESDF Map Text",
                    "ESDF Map", esdf_box_max + Vec3f(0, 0, 1.0), Color::Blue(),
                    0.6, 0);
      visualizeBoundingBox(mkr_arr, nh_->get_clock()->now().seconds(),
                           esdf_box_min, esdf_box_max, "ESDF Updating Range",
                           Color::Blue());
    }

    vm_.mkr_arr_pub->publish(mkr_arr);
  }

  void vecEVec3fToPC2(const vec_E<Vec3f> &points,
                      sensor_msgs::msg::PointCloud2 &cloud) {
    // 设置header信息
    pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
    pcl_cloud.resize(points.size());
    for (long unsigned int i = 0; i < points.size(); i++) {
      pcl_cloud[i].x = static_cast<float>(points[i][0]);
      pcl_cloud[i].y = static_cast<float>(points[i][1]);
      pcl_cloud[i].z = static_cast<float>(points[i][2]);
    }
    pcl::toROSMsg(pcl_cloud, cloud);
    cloud.header.stamp = nh_->get_clock()->now();
    cloud.header.frame_id = "world";
  }

public:
  typedef shared_ptr<ROGMapROS> Ptr;

  ROGMapROS(const rclcpp::Node::SharedPtr nh, const std::string &cfg_path)
      : nh_(nh) {
    // TODO: The current implementation uses a lenient QoS configuration for
    // message transmission.
    const rclcpp::QoS qos(
        rclcpp::QoS(1).best_effort().keep_last(1).durability_volatile());

    cfg_ = rog_map::Config(cfg_path);
    // 创建 TransformBroadcaster
    br_map_ego_ = std::make_shared<tf2_ros::TransformBroadcaster>(nh_);

    init();
    /// Initialize visualization module
    if (cfg_.visualization_en) {
      vm_.terrain_map_pub =
          nh_->create_publisher<sensor_msgs::msg::PointCloud2>(
              "rog_map/terrain_map", qos);

      vm_.occ_pub = nh_->create_publisher<sensor_msgs::msg::PointCloud2>(
          "rog_map/occ", qos);
      vm_.unknown_pub = nh_->create_publisher<sensor_msgs::msg::PointCloud2>(
          "rog_map/unk", qos);
      vm_.occ_inf_pub = nh_->create_publisher<sensor_msgs::msg::PointCloud2>(
          "rog_map/inf_occ", qos);
      vm_.unknown_inf_pub =
          nh_->create_publisher<sensor_msgs::msg::PointCloud2>(
              "rog_map/inf_unk", qos);

      if (cfg_.frontier_extraction_en) {
        vm_.frontier_pub = nh_->create_publisher<sensor_msgs::msg::PointCloud2>(
            "rog_map/frontier", qos);
      }

      if (cfg_.esdf_en) {
        vm_.esdf_pub = nh_->create_publisher<sensor_msgs::msg::PointCloud2>(
            "rog_map/esdf", qos);
        // vm_.esdf_neg_pub =
        // nh_->create_publisher<sensor_msgs::msg::PointCloud2>("rog_map/esdf/neg",
        // qos); vm_.esdf_occ_pub =
        // nh_->create_publisher<sensor_msgs::msg::PointCloud2>("rog_map/esdf/occ",
        // qos);
      }

      if (cfg_.viz_time_rate > 0) {
        const int cbk_dt_ms = static_cast<int>(1.0 / cfg_.viz_time_rate * 1000);
        vm_.viz_reen_cbk_group = nh_->create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive);
        vm_.viz_timer = nh_->create_wall_timer(
            std::chrono::milliseconds(cbk_dt_ms),
            std::bind(&ROGMapROS::vizCallback, this), vm_.viz_reen_cbk_group);
      }
    }

    vm_.mkr_arr_pub =
        nh_->create_publisher<visualization_msgs::msg::MarkerArray>(
            "rog_map/map_bound", qos);

    if (cfg_.ros_callback_en) {
      rc_.odom_me_cbk_group = nh_->create_callback_group(
          rclcpp::CallbackGroupType::MutuallyExclusive);
      rc_.cloud_me_cbk_group = nh_->create_callback_group(
          rclcpp::CallbackGroupType::MutuallyExclusive);
      rclcpp::SubscriptionOptions so;
      so.callback_group = rc_.odom_me_cbk_group;
      rc_.odom_sub = nh_->create_subscription<nav_msgs::msg::Odometry>(
          cfg_.odom_topic, qos,
          std::bind(&ROGMapROS::odomCallback, this, std::placeholders::_1), so);
      so.callback_group = rc_.cloud_me_cbk_group;
      rc_.cloud_sub = nh_->create_subscription<sensor_msgs::msg::PointCloud2>(
          cfg_.cloud_topic, qos,
          std::bind(&ROGMapROS::cloudCallback, this, std::placeholders::_1),
          so);
      rc_.update_cbk_group = nh_->create_callback_group(
          rclcpp::CallbackGroupType::MutuallyExclusive);
      rc_.update_timer = nh_->create_wall_timer(
          std::chrono::milliseconds(1), // 0.001秒，即1毫秒
          std::bind(&ROGMapROS::updateCallback, this), rc_.update_cbk_group);

      // 在构造函数中
      rc_.terrain_callback_group_ = nh_->create_callback_group(
          rclcpp::CallbackGroupType::MutuallyExclusive);

      Terrain::TerrainAnalyzer::Config terrain_cfg;
      terrain_cfg.resolution = cfg_.resolution;
      terrain_cfg.map_size_x = cfg_.terrain_map_size_x;
      terrain_cfg.map_size_y = cfg_.terrain_map_size_y;
      terrain_cfg.kernel_size = cfg_.terrain_kernel_size;
      terrain_cfg.max_step_height = cfg_.terrain_max_step_height;
      terrain_cfg.robot_height = cfg_.terrain_robot_height;
      terrain_cfg.steep_threshold = cfg_.terrain_steep_threshold;
      terrain_analyzer_.setConfig(terrain_cfg);

      rc_.terrain_timer_ = nh_->create_wall_timer(
          std::chrono::milliseconds(
              static_cast<int>(1000.0 / std::max(cfg_.terrain_time_rate, 1.0))),
          std::bind(&ROGMapROS::terrainCallback, this),
          rc_.terrain_callback_group_);
    }

    if (cfg_.global_map_en) {
      vm_.global_3docc_pub =
          nh_->create_publisher<sensor_msgs::msg::PointCloud2>(
              "rog_map/global_3docc", qos);
      vm_.global_terrain_pub =
          nh_->create_publisher<sensor_msgs::msg::PointCloud2>(
              "rog_map/global_terrain_map", qos);
      loadGlobalPCD();

      rc_.global_map_callback_group_ = nh_->create_callback_group(
          rclcpp::CallbackGroupType::MutuallyExclusive);
      rc_.global_map_timer_ = nh_->create_wall_timer(
          std::chrono::milliseconds(static_cast<int>(
              1000.0 / std::max(cfg_.global_map_time_rate, 1.0))),
          std::bind(&ROGMapROS::publishGlobalMap, this),
          rc_.global_map_callback_group_);
      RCLCPP_INFO(nh_->get_logger(),
                  "Global map publisher initialized, publishing at %.1f Hz",
                  cfg_.global_map_time_rate);
    }

    // Save-map service
    if (cfg_.save_map_en) {
      rc_.save_map_srv_ = nh_->create_service<std_srvs::srv::Trigger>(
          "rog_map/save_map",
          std::bind(&ROGMapROS::saveMapCallback, this, std::placeholders::_1,
                    std::placeholders::_2));
      RCLCPP_INFO(nh_->get_logger(),
                  "Save-map service ready at /rog_map/save_map");
    }
  }

private:
  static void visualizeBoundingBox(visualization_msgs::msg::MarkerArray &mkrarr,
                                   const double &stamp, const Vec3f &box_min,
                                   const Vec3f &box_max, const string &ns,
                                   const Color &color,
                                   const double &size_x = 0.1,
                                   const double &alpha = 1.0,
                                   const bool &print_ns = true) {
    Vec3f size = (box_max - box_min) / 2;
    Vec3f vis_pos_world = (box_min + box_max) / 2;
    double width = size.x();
    double length = size.y();
    double hight = size.z();

    // Publish Bounding box
    int id = 0;
    visualization_msgs::msg::Marker line_strip;
    line_strip.header.stamp = rclcpp::Time(stamp);
    line_strip.header.frame_id = "world";
    line_strip.action = visualization_msgs::msg::Marker::ADD;
    line_strip.ns = ns;
    line_strip.pose.orientation.w = 1.0;
    line_strip.id = id++; // unique id, useful when multiple markers exist.
    line_strip.type = visualization_msgs::msg::Marker::LINE_STRIP; // marker
                                                                   // type
    line_strip.scale.x = size_x;

    line_strip.color = color;
    line_strip.color.a = alpha; //不透明度，设0则全透明
    geometry_msgs::msg::Point p[8];

    // vis_pos_world是目标物的坐标
    p[0].x = vis_pos_world(0) - width;
    p[0].y = vis_pos_world(1) + length;
    p[0].z = vis_pos_world(2) + hight;
    p[1].x = vis_pos_world(0) - width;
    p[1].y = vis_pos_world(1) - length;
    p[1].z = vis_pos_world(2) + hight;
    p[2].x = vis_pos_world(0) - width;
    p[2].y = vis_pos_world(1) - length;
    p[2].z = vis_pos_world(2) - hight;
    p[3].x = vis_pos_world(0) - width;
    p[3].y = vis_pos_world(1) + length;
    p[3].z = vis_pos_world(2) - hight;
    p[4].x = vis_pos_world(0) + width;
    p[4].y = vis_pos_world(1) + length;
    p[4].z = vis_pos_world(2) - hight;
    p[5].x = vis_pos_world(0) + width;
    p[5].y = vis_pos_world(1) - length;
    p[5].z = vis_pos_world(2) - hight;
    p[6].x = vis_pos_world(0) + width;
    p[6].y = vis_pos_world(1) - length;
    p[6].z = vis_pos_world(2) + hight;
    p[7].x = vis_pos_world(0) + width;
    p[7].y = vis_pos_world(1) + length;
    p[7].z = vis_pos_world(2) + hight;
    // LINE_STRIP类型仅仅将line_strip.points中相邻的两个点相连，如0和1，1和2，2和3
    for (int i = 0; i < 8; i++) {
      line_strip.points.push_back(p[i]);
    }
    //为了保证矩形框的八条边都存在：
    line_strip.points.push_back(p[0]);
    line_strip.points.push_back(p[3]);
    line_strip.points.push_back(p[2]);
    line_strip.points.push_back(p[5]);
    line_strip.points.push_back(p[6]);
    line_strip.points.push_back(p[1]);
    line_strip.points.push_back(p[0]);
    line_strip.points.push_back(p[7]);
    line_strip.points.push_back(p[4]);
    mkrarr.markers.push_back(line_strip);
  }

  static void visualizeText(visualization_msgs::msg::MarkerArray &mkr_arr,
                            const double &stamp, const std::string &ns,
                            const std::string &text, const Vec3f &position,
                            const Color &c = Color::White(),
                            const double &size = 0.6, const int &id = -1) {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "world";
    marker.header.stamp = rclcpp::Time(stamp);
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.ns = ns.c_str();
    if (id >= 0) {
      marker.id = id;
    } else {
      static int id = 0;
      marker.id = id++;
    }
    marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    marker.scale.z = size;
    marker.color = c;
    marker.text = text;
    marker.pose.position.x = position.x();
    marker.pose.position.y = position.y();
    marker.pose.position.z = position.z();
    marker.pose.orientation.w = 1.0;
    mkr_arr.markers.push_back(marker);
  };

  static void visualizePoint(visualization_msgs::msg::MarkerArray &mkr_arr,
                             const double &stamp, const Vec3f &pt,
                             Color color = Color::Pink(), std::string ns = "pt",
                             double size = 0.1, int id = -1,
                             const bool &print_ns = true) {
    visualization_msgs::msg::Marker marker_ball;
    static int cnt = 0;
    Vec3f cur_pos = pt;
    if (isnan(pt.x()) || isnan(pt.y()) || isnan(pt.z())) {
      return;
    }
    marker_ball.header.frame_id = "world";
    marker_ball.header.stamp = rclcpp::Time(stamp);
    marker_ball.ns = ns.c_str();
    marker_ball.id = id >= 0 ? id : cnt++;
    marker_ball.action = visualization_msgs::msg::Marker::ADD;
    marker_ball.pose.orientation.w = 1.0;
    marker_ball.type = visualization_msgs::msg::Marker::SPHERE;
    marker_ball.scale.x = size;
    marker_ball.scale.y = size;
    marker_ball.scale.z = size;
    marker_ball.color = color;

    geometry_msgs::msg::Point p;
    p.x = cur_pos.x();
    p.y = cur_pos.y();
    p.z = cur_pos.z();

    marker_ball.pose.position = p;
    mkr_arr.markers.push_back(marker_ball);

    // add test
    if (print_ns) {
      visualization_msgs::msg::Marker marker;
      marker.header.frame_id = "world";
      marker.header.stamp = rclcpp::Time(stamp);
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.pose.orientation.w = 1.0;
      marker.ns = ns + "_text";
      if (id >= 0) {
        marker.id = id;
      } else {
        static int id = 0;
        marker.id = id++;
      }
      marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      marker.scale.z = 0.6;
      marker.color = color;
      marker.text = ns;
      marker.pose.position.x = cur_pos.x();
      marker.pose.position.y = cur_pos.y();
      marker.pose.position.z = cur_pos.z() + 0.5;
      marker.pose.orientation.w = 1.0;
      mkr_arr.markers.push_back(marker);
    }
  }
};
} // namespace rog_map
#endif // ROG_MAP_ROS_HPP
