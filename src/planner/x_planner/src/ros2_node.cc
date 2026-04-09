#include "../include/ros2_node.h"
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>
namespace ros2 {

// void XPlannerROS2::odomCallback(const carstatemsgs::CarState::ConstPtr &msg){
//   map_->sdf_map_->odom_ = *msg;
//   map_->sdf_map_->has_odom_ = true;
// }

void XPlannerROS2::pointCloudCallback(
    const sensor_msgs::msg::PointCloud2::SharedPtr &msg) {
  static auto clock = std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME);
  static tf2_ros::Buffer tf_buffer(clock);
  static tf2_ros::TransformListener tf_listener(tf_buffer);

  // Get transformation information
  geometry_msgs::msg::TransformStamped transformStamped;
  try {
    transformStamped = tf_buffer.lookupTransform(
        "world", msg->header.frame_id, msg->header.stamp,
        rclcpp::Duration::from_seconds(0.1));
  } catch (tf2::TransformException &ex) {
    RCLCPP_WARN(node_->get_logger(), "%s", ex.what());
    return;
  }

  map_->sdf_map_->odom_pos_.head(2) =
      Eigen::Vector2d(transformStamped.transform.translation.x,
                      transformStamped.transform.translation.y);
  // TODO :优化
  auto &q = transformStamped.transform.rotation;
  double yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                          1.0 - 2.0 * (q.y * q.y + q.z * q.z));
  map_->sdf_map_->odom_pos_[2] = yaw;

  // Perform coordinate transformation
  sensor_msgs::msg::PointCloud2 transformed_cloud;
  tf2::doTransform(*msg, transformed_cloud, transformStamped);

  // Convert to PCL format
  pcl::fromROSMsg(transformed_cloud, map_->sdf_map_->cloud_);

  map_->sdf_map_->occ_need_update_ = true;
}

void XPlannerROS2::publish_gridmap() {
  pcl::PointCloud<pcl::PointXYZI> cloud_vis;
  sensor_msgs::msg::PointCloud2 map_vis;
  for (int idx = 1; idx < map_->sdf_map_->GLXY_SIZE_; idx++) {
    // if(gridmap_[idx]==Unoccupied){
    //   Eigen::Vector2d corrd = gridIndex2coordd(vectornum2gridIndex(idx));
    //   pcl::PointXYZI pt;
    //   pt.x = corrd.x(); pt.y = corrd.y(); pt.z = 0.1;
    //   pt.intensity = 8.0;
    //   cloud_vis.points.push_back(pt);
    // }
    if (map_->sdf_map_->gridmap_[idx] == map_->sdf_map_->Occupied) {
      Eigen::Vector2d corrd = map_->sdf_map_->gridIndex2coordd(
          map_->sdf_map_->vectornum2gridIndex(idx));
      pcl::PointXYZI pt;
      pt.x = corrd.x();
      pt.y = corrd.y();
      pt.z = 0.1;
      pt.intensity = 0.0;
      cloud_vis.points.push_back(pt);
    }
    if (map_->sdf_map_->gridmap_[idx] == map_->sdf_map_->Unknown) {
      Eigen::Vector2d corrd = map_->sdf_map_->gridIndex2coordd(
          map_->sdf_map_->vectornum2gridIndex(idx));
      pcl::PointXYZI pt;
      pt.x = corrd.x();
      pt.y = corrd.y();
      pt.z = 0.1;
      pt.intensity = 8.0;
      cloud_vis.points.push_back(pt);
    }
  }

  pcl::PointXYZI pt;
  pt.x = 100.0;
  pt.y = 100.0;
  pt.z = 0.1;
  pt.intensity = 10.0;
  cloud_vis.points.push_back(pt);

  cloud_vis.width = cloud_vis.points.size();
  cloud_vis.height = 1;
  cloud_vis.is_dense = true;
  pcl::toROSMsg(cloud_vis, map_vis);
  map_vis.header.frame_id = "world";
  pub_gridmap_->publish(map_vis);
}
void XPlannerROS2::publish_ESDF() {
  pcl::PointCloud<pcl::PointXYZI> cloud_vis;
  sensor_msgs::msg::PointCloud2 surf_vis;
  const double min_dist = 0.0;
  const double max_dist = 5.0;
  int size = map_->sdf_map_->distance_buffer_all_.size();
  for (int i = 1; i < size; i++) {
    Eigen::Vector2d coord =
        map_->sdf_map_->gridIndex2coordd(map_->sdf_map_->vectornum2gridIndex(i));
    pcl::PointXYZI pt;
    pt.x = coord.x();
    pt.y = coord.y();
    pt.z = 0.0;
    pt.intensity = std::max(
        min_dist, std::min(map_->sdf_map_->distance_buffer_all_[i], max_dist));
    cloud_vis.points.push_back(pt);
  }
  cloud_vis.width = cloud_vis.points.size();
  cloud_vis.height = 1;
  cloud_vis.is_dense = true;
  pcl::toROSMsg(cloud_vis, surf_vis);
  surf_vis.header.frame_id = "world";
  pub_ESDF_->publish(surf_vis);
}
void XPlannerROS2::publish_ESDFGrad(){
    visualization_msgs::msg::MarkerArray grad_all;
    
    Eigen::Vector2i min_cut(0,0);
    Eigen::Vector2i max_cut(map_->sdf_map_->GLX_SIZE_-1, map_->sdf_map_->GLY_SIZE_-1);

    for (int x = min_cut(0)+2; x < max_cut(0); x+=5)
      for (int y = min_cut(1)+2; y < max_cut(1); y+=5) {
        Eigen::Vector2d pos = map_->sdf_map_->gridIndex2coordd(x,y);        
        Eigen::Vector2d d(0.025,0.025);
        pos = pos + d;
        Eigen::Vector2d grad;
        map_->sdf_map_->getDistWithGradBilinear(pos,grad);
        visualization_msgs::msg::Marker grad_Marker;
        grad_Marker.header.frame_id = "world";
        grad_Marker.header.stamp = node_->now();
        grad_Marker.ns = "world";
        grad_Marker.type = visualization_msgs::msg::Marker::ARROW;
        grad_Marker.action = visualization_msgs::msg::Marker::ADD;
        grad_Marker.id = (x+1) + (y*100000);
        grad_Marker.pose.position.x = pos[0];
        grad_Marker.pose.position.y = pos[1];
        grad_Marker.pose.position.z = 0.0;
        
        Eigen::Quaterniond Quat;
        Eigen::Vector3d vectorBefore(1, 0, 0);
        Eigen::Vector3d vectorAfter(grad.x(), grad.y(), 0.0);
        Quat = Eigen::Quaterniond::FromTwoVectors(vectorBefore, vectorAfter);

        grad_Marker.pose.orientation.w = Quat.w();
        grad_Marker.pose.orientation.x = Quat.x();
        grad_Marker.pose.orientation.y = Quat.y();
        grad_Marker.pose.orientation.z = Quat.z();

        grad_Marker.scale.x = (grad.norm()+0.01)/10.0;
        if(grad.norm()>1000){
          RCLCPP_INFO(node_->get_logger(),"grad error!!!  position: %f  %f    grad:%f  %f   grad.norm(): %f",pos.x(),pos.y(),grad.x(),grad.y(),grad.norm());
        }
        grad_Marker.scale.y = 0.01;
        grad_Marker.scale.z = 0.01;

        grad_Marker.color.r = 0.0f;
        grad_Marker.color.g = 1.0f;
        grad_Marker.color.b = 0.0f;
        grad_Marker.color.a = 1.0;
        grad_Marker.lifetime = rclcpp::Duration(0.0,100.0);
        grad_Marker.frame_locked = true;

        grad_all.markers.push_back(grad_Marker);
      }

    pub_gradESDF_->publish(grad_all);

}
} // namespace ros2