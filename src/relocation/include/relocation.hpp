#pragma once
#include "fmt_eigen.hpp"
#include <memory>
#include <optional>
#include <pcl/features/fpfh_omp.h>      // FPFH加速计算
#include <pcl/features/normal_3d_omp.h> // 使用多线程加速法向量估计
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/voxel_grid.h> // 体素滤波器，用于点云下采样
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/ia_ransac.h> // SAC-IA初始配准算法
#include <pcl/registration/ndt.h>       //NDT 配准算法
                                        // FGR

#include "small_gicp/ann/kdtree_omp.hpp"
#include "small_gicp/factors/gicp_factor.hpp"
#include "small_gicp/pcl/pcl_point.hpp"
#include "small_gicp/registration/reduction_omp.hpp"
#include "small_gicp/registration/registration.hpp"

#include <small_gicp/pcl/pcl_point_traits.hpp>
#include <utility>
namespace relocation {
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
typedef pcl::PointCloud<pcl::Normal> pointnormal;          // 法向量类型
typedef pcl::PointCloud<pcl::FPFHSignature33> fpfhFeature; // FPFH特征类型

/**
@brief: 重定位的工具类
*/
class Relocation {
public:
  Relocation() {
    tree_.reset(new pcl::search::KdTree<pcl::PointXYZ>());

    gicp_.register_ = std::make_shared<small_gicp::Registration<
        small_gicp::GICPFactor, small_gicp::ParallelReductionOMP>>();
  };

public:
  auto fgr(PointCloud::Ptr source_cloud);
  auto sac_ia(PointCloud::Ptr source_cloud, PointCloud::Ptr target_cloud)
      -> std::pair<double, Eigen::Matrix4f>;
  auto sac_ia(PointCloud::Ptr source_cloud)
      -> std::optional<std::pair<double, Eigen::Matrix4f>>;
  auto ndt(PointCloud::Ptr source_cloud, Eigen::Matrix4f init_guess)
      -> std::pair<double, Eigen::Matrix4f>;
  auto init_target(PointCloud::Ptr target_cloud, std::string model) -> void;
  auto gicp(PointCloud::Ptr source_cloud, PointCloud::Ptr target_cloud)
      -> std::optional<Eigen::Isometry3d>;
  auto gicp(PointCloud::Ptr source_cloud,
            std::optional<Eigen::Isometry3d> &initial_pose)
      -> std::optional<Eigen::Isometry3d>;

private:
  // 计算FPFH特征的函数
  auto compute_fpfh_feature(PointCloud::Ptr input_cloud,
                            pcl::search::KdTree<pcl::PointXYZ>::Ptr tree)
      -> fpfhFeature::Ptr;
  // 计算FPFH特征的函数
  auto compute_fpfh_feature_kd(PointCloud::Ptr input_cloud,
                               pcl::search::KdTree<pcl::PointXYZ>::Ptr tree)
      -> fpfhFeature::Ptr;

public:
  struct RelocationConfig {
    // GICP
    int gicp_num_neighbors_ = 10, gicp_num_threads_ = 4;
    float gicp_max_dist_sq_ = 2.0;
    float registered_leaf_size_ = 0.25f;
    float global_leaf_size_ = 0.25f;
    int gicp_max_iterations = 60;
    // FPFH
    int fpfh_threads = 8;
    float fpfh_normal_radius = 0.5;
    float fpfh_estimator_radius = 1.0;
    // SAC-IA
    //  迭代次数（SAC-IA默认200）
    int sac_iterations = 200;
    // 设置样本点之间的最小距离
    float sac_min_sample_distance = 0.1;
    // 设置邻居数量，用于随机特征对应选择
    int sac_k_num = 6;
    //ndt
    float ndt_voxel_filter_size=0.2;
    float ndt_epsilon=0.01;
    float ndt_step_size=0.1;
    float ndt_resolution=1.0;
    int ndt_maximum_iterations=35;
  };

private:
  RelocationConfig config_;
  struct {
    std::shared_ptr<small_gicp::KdTree<pcl::PointCloud<pcl::PointCovariance>>>
        target_tree_;
    std::shared_ptr<small_gicp::KdTree<pcl::PointCloud<pcl::PointCovariance>>>
        source_tree_;
    std::shared_ptr<small_gicp::Registration<small_gicp::GICPFactor,
                                             small_gicp::ParallelReductionOMP>>
        register_;
    pcl::PointCloud<pcl::PointCovariance>::Ptr target_;
    pcl::PointCloud<pcl::PointCovariance>::Ptr source_;

  } gicp_;
  PointCloud::Ptr target_cloud_;
  struct {
    pcl::SampleConsensusInitialAlignment<pcl::PointXYZ, pcl::PointXYZ,
                                         pcl::FPFHSignature33>
        sac_ia;

    fpfhFeature::Ptr target_fpfh_;
  } rough_match_;
  struct {
    pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;
  } ndt_user_;
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree_;

public:
  auto init_param(RelocationConfig config) -> void { this->config_ = config; };
};
} // namespace relocation