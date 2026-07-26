#pragma once

#include "pcl/filters/voxel_grid.h"
#include "pcl/impl/point_types.hpp"
#include "pcl/io/pcd_io.h"
#include "pcl/point_cloud.h"
#include <Eigen/Core>
#include <optional>
#include <spdlog/spdlog.h>
namespace utils {

template <typename PointT>
inline typename pcl::PointCloud<PointT>::Ptr
voxel_filter(const typename pcl::PointCloud<PointT>::Ptr &cloud,
             const Eigen::Vector3f &leaf_size) {
  typename pcl::PointCloud<PointT>::Ptr cloud_filtered(
      new pcl::PointCloud<PointT>);
  pcl::VoxelGrid<PointT> sor;
  sor.setInputCloud(cloud);
  sor.setLeafSize(leaf_size.x(), leaf_size.y(), leaf_size.z());
  sor.filter(*cloud_filtered);
  return cloud_filtered;
}
inline auto remove_nan_points(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud)
    -> void {
  // 移除输入点云中的NAN点
  std::vector<int> indices;
  pcl::removeNaNFromPointCloud(*input_cloud, *input_cloud,
                               indices); // 去除NAN并保存有效索引
}
inline auto read_pcd(std::string pcd_load_name)
    -> std::optional<pcl::PointCloud<pcl::PointXYZ>::Ptr> {
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

  if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_load_name, *cloud) ==
      -1) //* load the file
  {
    spdlog::error("Couldn't read file pcd");
    return std::nullopt;
  }
  spdlog::info(
      "Loaded:{} data points from pcd with the following fields",
      cloud->width * cloud->height);
  return cloud;
}
} // namespace utils