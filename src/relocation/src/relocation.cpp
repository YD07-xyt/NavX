#include "relocation.hpp"
#include <optional>
#include <small_gicp/util/downsampling_omp.hpp>
#include <small_gicp/util/normal_estimation_omp.hpp>

#include <spdlog/spdlog.h>
#include <utility>

namespace relocation {

auto Relocation::sac_ia(PointCloud::Ptr source_cloud,
                        PointCloud::Ptr target_cloud)
    -> std::pair<double, Eigen::Matrix4f> {
  /**
      输入的是 去除源点云和目标点云中的NAN点，下采样后的点云
  */

  // 计算源点云和目标点云的FPFH特征
  fpfhFeature::Ptr source_fpfh = compute_fpfh_feature(source_cloud, tree_);
  fpfhFeature::Ptr target_fpfh = compute_fpfh_feature(target_cloud, tree_);

  // 执行SAC-IA配准
  // 使用SAC-IA进行点云配准

  rough_match_.sac_ia.setInputSource(source_cloud); // 设置源点云
  rough_match_.sac_ia.setSourceFeatures(source_fpfh); // 设置源点云的FPFH特征
  rough_match_.sac_ia.setInputTarget(target_cloud); // 设置目标点云
  rough_match_.sac_ia.setTargetFeatures(target_fpfh); // 设置目标点云的FPFH特征
  rough_match_.sac_ia.setMaximumIterations(
      config_.sac_iterations); // 迭代次数（SAC-IA默认200）
  rough_match_.sac_ia.setMinSampleDistance(
      config_.sac_min_sample_distance); // 设置样本点之间的最小距离
  rough_match_.sac_ia.setCorrespondenceRandomness(
      config_.sac_k_num); // 设置邻居数量，用于随机特征对应选择
  PointCloud::Ptr aligned_cloud(new PointCloud); // 用于存储配准后的点云
  rough_match_.sac_ia.align(*aligned_cloud);     // 执行配准

  // 输出配准结果
  auto score = rough_match_.sac_ia.getFitnessScore();
  auto tf = rough_match_.sac_ia.getFinalTransformation();
  spdlog::info("[SAC_IA]: success");
  spdlog::info("[SAC_IA]:配准得分: {}", score);
  spdlog::info("[SAC_IA]:变换矩阵： {}", tf);
  return std::make_pair(score, tf);
};

auto Relocation::sac_ia(PointCloud::Ptr source_cloud)
    -> std::optional<std::pair<double, Eigen::Matrix4f>> {
  /**
      输入的是 去除源点云和目标点云中的NAN点，下采样后的点云
  */
  spdlog::info("[SAC-IA]");
  // 计算源点云和目标点云的FPFH特征
  fpfhFeature::Ptr source_fpfh = compute_fpfh_feature(source_cloud, tree_);
  rough_match_.target_fpfh_ = compute_fpfh_feature(target_cloud_, tree_);

  // 执行SAC-IA配准
  // 使用SAC-IA进行点云配准

  rough_match_.sac_ia.setInputSource(source_cloud); // 设置源点云
  rough_match_.sac_ia.setSourceFeatures(source_fpfh); // 设置源点云的FPFH特征
  rough_match_.sac_ia.setInputTarget(target_cloud_); // 设置目标点云
  rough_match_.sac_ia.setTargetFeatures(
      rough_match_.target_fpfh_); // 设置目标点云的FPFH特征

  rough_match_.sac_ia.setMaximumIterations(
      config_.sac_iterations); // 迭代次数（SAC-IA默认200）

  rough_match_.sac_ia.setMinSampleDistance(
      config_.sac_min_sample_distance); // 设置样本点之间的最小距离
  rough_match_.sac_ia.setCorrespondenceRandomness(
      config_.sac_k_num); // 设置邻居数量，用于随机特征对应选择
  PointCloud::Ptr aligned_cloud(new PointCloud); // 用于存储配准后的点云
  rough_match_.sac_ia.align(*aligned_cloud);     // 执行配准

  // 输出配准结果
  auto score = rough_match_.sac_ia.getFitnessScore();
  auto tf = rough_match_.sac_ia.getFinalTransformation();
  spdlog::info("[SAC_IA]: success");
  spdlog::info("[SAC_IA]:配准得分: {}", score);
  spdlog::info("[SAC_IA]:变换矩阵： {}", tf);
  // 在取得 score 和 tf 后，额外做物理约束
  Eigen::Vector3f trans = tf.block<3, 1>(0, 3);
  float translation_norm = trans.norm();

  // 假设地图半径 50m，机器人不可能瞬间移动到 30m 外
  if (translation_norm > 3.0) {
    spdlog::warn("[SAC-IA] 丢弃异常平移: {} m", translation_norm);
    return std::nullopt; // 返回无效结果
  }
  return std::make_pair(score, tf);
};

auto Relocation::fgr(PointCloud::Ptr source_cloud){};

void Relocation::init_target(PointCloud::Ptr target_cloud, std::string model) {
  if (model == "gicp") {
    // 使用传入的 target_cloud，而非 rough_match_.target_cloud_
    gicp_.target_ = small_gicp::voxelgrid_sampling_omp<
        pcl::PointCloud<pcl::PointXYZ>, pcl::PointCloud<pcl::PointCovariance>>(
        *target_cloud, config_.global_leaf_size_);
    small_gicp::estimate_covariances_omp(
        *gicp_.target_, config_.gicp_num_neighbors_, config_.gicp_num_threads_);
    gicp_.target_tree_ = std::make_shared<
        small_gicp::KdTree<pcl::PointCloud<pcl::PointCovariance>>>(
        gicp_.target_, small_gicp::KdTreeBuilderOMP(config_.gicp_num_threads_));
  } else if (model == "sac_ia") {
    target_cloud_ = target_cloud;
  } else if (model == "ndt") {
    target_cloud_ = target_cloud;
  }
}

auto Relocation::gicp(PointCloud::Ptr source_cloud,
                      std::optional<Eigen::Isometry3d> &initial_pose)
    -> std::optional<Eigen::Isometry3d> {
  spdlog::info("[GICP]");
  /** param */
  if (!initial_pose.has_value()) {
    spdlog::info("[GICP] default init pose ");
    initial_pose = Eigen::Isometry3d::Identity();
  }
  gicp_.source_ =
      small_gicp::voxelgrid_sampling_omp<pcl::PointCloud<pcl::PointXYZ>,
                                         pcl::PointCloud<pcl::PointCovariance>>(
          *source_cloud, config_.registered_leaf_size_);

  small_gicp::estimate_covariances_omp(
      *gicp_.source_, config_.gicp_num_neighbors_, config_.gicp_num_threads_);

  gicp_.source_tree_ = std::make_shared<
      small_gicp::KdTree<pcl::PointCloud<pcl::PointCovariance>>>(
      gicp_.source_, small_gicp::KdTreeBuilderOMP(config_.gicp_num_threads_));

  if (!gicp_.source_ || !gicp_.source_tree_) {
    return std::nullopt;
  }

  gicp_.register_->reduction.num_threads = config_.gicp_num_threads_;
  gicp_.register_->rejector.max_dist_sq = config_.gicp_max_dist_sq_;
  gicp_.register_->optimizer.max_iterations = config_.gicp_max_iterations;

  auto result = gicp_.register_->align(*gicp_.target_, *gicp_.source_,
                                       *gicp_.target_tree_, *initial_pose);

  if (result.converged) {
    spdlog::info("[GICP]  success converge");
    return result.T_target_source;
  } else {
    spdlog::warn("[GICP]: did not converge");
  }
  return std::nullopt;
};
auto Relocation::compute_fpfh_feature(
    PointCloud::Ptr input_cloud, pcl::search::KdTree<pcl::PointXYZ>::Ptr tree)
    -> fpfhFeature::Ptr {
  // 1. 估计法向量（使用半径搜索，自适应点云密度）
  pointnormal::Ptr normals(new pointnormal);
  pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne;
  ne.setInputCloud(input_cloud);
  ne.setNumberOfThreads(config_.fpfh_threads);
  ne.setSearchMethod(tree);

  // 关键改动：使用半径搜索，替代固定K搜索
  // 半径建议为体素滤波尺寸的2~3倍，这里按0.25m体素设为0.5m
  ne.setRadiusSearch(config_.fpfh_normal_radius);
  ne.compute(*normals);

  // 可选：根据视点修正法线方向（如果传感器位置已知）
  // 如果你的点云坐标系原点大致为传感器位置，可设置为(0,0,0)
  // ne.setViewPoint(0.0, 0.0, 0.0); // 根据实际情况调整

  // 输出法线有效性检查（调试用）
  int valid_normals = 0;
  for (const auto &n : normals->points) {
    if (pcl::isFinite(n))
      valid_normals++;
  }
  spdlog::info("Valid normals: {}/{}", valid_normals, normals->size());

  // 2. 计算 FPFH 特征
  fpfhFeature::Ptr fpfh(new fpfhFeature);
  pcl::FPFHEstimationOMP<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33>
      fpfh_estimator;
  fpfh_estimator.setNumberOfThreads(config_.fpfh_threads);
  fpfh_estimator.setInputCloud(input_cloud);
  fpfh_estimator.setInputNormals(normals);
  fpfh_estimator.setSearchMethod(tree);

  // 同样使用半径搜索，半径通常比法线估计更大（如0.8~1.0m）
  fpfh_estimator.setRadiusSearch(config_.fpfh_estimator_radius);
  fpfh_estimator.compute(*fpfh);

  return fpfh;
}
// 计算FPFH特征的函数
auto Relocation::compute_fpfh_feature_kd(
    PointCloud::Ptr input_cloud, pcl::search::KdTree<pcl::PointXYZ>::Ptr tree)
    -> fpfhFeature::Ptr {

  // 1. 估计法向量
  pointnormal::Ptr normals(new pointnormal); // 用于存储计算得到的法向量
  pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal>
      ne;                        // 使用OMP加速法向量计算
  ne.setInputCloud(input_cloud); // 设置输入点云
  ne.setNumberOfThreads(8);      // 设置并行计算的线程数
  ne.setSearchMethod(tree);      // 使用KD树加速搜索
  ne.setKSearch(10);             // 设置K近邻点数量为10
  ne.compute(*normals);          // 计算法向量

  // 2. 计算FPFH特征
  fpfhFeature::Ptr fpfh(new fpfhFeature); // 创建FPFH特征存储容器
  pcl::FPFHEstimationOMP<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33>
      fpfh_estimator;                        // FPFH特征估计器
  fpfh_estimator.setNumberOfThreads(8);      // 设置并行计算线程数
  fpfh_estimator.setInputCloud(input_cloud); // 设置输入点云
  fpfh_estimator.setInputNormals(normals);   // 设置法向量
  fpfh_estimator.setSearchMethod(tree);      // 使用KD树加速搜索
  fpfh_estimator.setKSearch(10);             // 设置K近邻点数量为10
  fpfh_estimator.compute(*fpfh);             // 计算FPFH特征

  return fpfh;
}
auto Relocation::gicp(PointCloud::Ptr source_cloud,
                      PointCloud::Ptr target_cloud)
    -> std::optional<Eigen::Isometry3d> {
  /** param */
  int num_neighbors_ = 10, num_threads_ = 4, max_dist_sq_ = 1.0;
  float registered_leaf_size_ = 0.25;
  float global_leaf_size_ = 0.25;
  Eigen::Isometry3d result_t;
  Eigen::Isometry3d previous_result_t;

  // Downsample points and convert them into
  // pcl::PointCloud<pcl::PointCovariance>
  gicp_.target_ =
      small_gicp::voxelgrid_sampling_omp<pcl::PointCloud<pcl::PointXYZ>,
                                         pcl::PointCloud<pcl::PointCovariance>>(
          *target_cloud, global_leaf_size_);
  gicp_.source_ =
      small_gicp::voxelgrid_sampling_omp<pcl::PointCloud<pcl::PointXYZ>,
                                         pcl::PointCloud<pcl::PointCovariance>>(
          *source_cloud, registered_leaf_size_);

  // Estimate covariances of points
  small_gicp::estimate_covariances_omp(*gicp_.target_, num_neighbors_,
                                       num_threads_);

  small_gicp::estimate_covariances_omp(*gicp_.source_, num_neighbors_,
                                       num_threads_);

  // Create KdTree for target
  gicp_.target_tree_ = std::make_shared<
      small_gicp::KdTree<pcl::PointCloud<pcl::PointCovariance>>>(
      gicp_.target_, small_gicp::KdTreeBuilderOMP(num_threads_));

  gicp_.source_tree_ = std::make_shared<
      small_gicp::KdTree<pcl::PointCloud<pcl::PointCovariance>>>(
      gicp_.source_, small_gicp::KdTreeBuilderOMP(num_threads_));

  if (!gicp_.source_ || !gicp_.source_tree_) {
    return std::nullopt;
  }

  gicp_.register_->reduction.num_threads = num_threads_;
  gicp_.register_->rejector.max_dist_sq = max_dist_sq_;
  gicp_.register_->optimizer.max_iterations = 10;

  auto result = gicp_.register_->align(*gicp_.target_, *gicp_.source_,
                                       *gicp_.target_tree_, previous_result_t);

  if (result.converged) {
    result_t = previous_result_t = result.T_target_source;
  } else {
    spdlog::warn("[GICP]: did not converge");
  }
  return result_t;
};
auto Relocation::ndt(PointCloud::Ptr source_cloud, Eigen::Matrix4f init_guess)
    -> std::pair<double, Eigen::Matrix4f> {
  // Filtering input scan
  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(
      new pcl::PointCloud<pcl::PointXYZ>);
  pcl::ApproximateVoxelGrid<pcl::PointXYZ> approximate_voxel_filter;
  approximate_voxel_filter.setLeafSize(config_.ndt_voxel_filter_size,
                                       config_.ndt_voxel_filter_size,
                                       config_.ndt_voxel_filter_size);
  approximate_voxel_filter.setInputCloud(source_cloud);
  approximate_voxel_filter.filter(*filtered_cloud);
  spdlog::info("Filtered cloud contains: {} data points",
               filtered_cloud->size());
  ndt_user_.ndt.setTransformationEpsilon(config_.ndt_epsilon);
  ndt_user_.ndt.setStepSize(config_.ndt_step_size);
  ndt_user_.ndt.setResolution(config_.ndt_resolution);
  ndt_user_.ndt.setMaximumIterations(config_.ndt_maximum_iterations);

  ndt_user_.ndt.setInputSource(filtered_cloud);
  ndt_user_.ndt.setInputTarget(target_cloud_); // target_cloud_ 是成员变量

  pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud(
      new pcl::PointCloud<pcl::PointXYZ>);
  ndt_user_.ndt.align(*output_cloud, init_guess);

  spdlog::info("NDT has {}, score: {}",
               (ndt_user_.ndt.hasConverged() ? "converged" : "not converged"),
               ndt_user_.ndt.getFitnessScore());

  auto score = ndt_user_.ndt.getFitnessScore();
  auto tf = ndt_user_.ndt.getFinalTransformation();
  return std::make_pair(score, tf);
}
} // namespace relocation