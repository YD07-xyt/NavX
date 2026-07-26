#include "cloud_preprocess_node.hpp"

#include <spdlog/spdlog.h>

#include <rclcpp/rclcpp.hpp>

namespace cloud_preprocess {

CloudPreprocessNode::CloudPreprocessNode(const rclcpp::NodeOptions &options)
    : rclcpp::Node("cloud_preprocess_node", options) {
  loadParams();

  sub_cloud_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      topic_input_cloud_, 10,
      std::bind(&CloudPreprocessNode::cloudCallback, this,
                std::placeholders::_1));

  pub_obstacles_ =
      create_publisher<sensor_msgs::msg::PointCloud2>(topic_obstacles_, 10);
  pub_ground_ =
      create_publisher<sensor_msgs::msg::PointCloud2>(topic_ground_, 10);
  pub_inflated_ =
      create_publisher<sensor_msgs::msg::PointCloud2>(topic_inflated_, 10);

  spdlog::info("[cloud_preprocess_node] 节点已启动，订阅: {} / 发布障碍: {}",
               topic_input_cloud_, topic_obstacles_);
}

void CloudPreprocessNode::loadParams() {
  Params p = core_.params();
  // 算法参数
  declare_parameter("enable_outlier", p.enable_outlier);
  declare_parameter("outlier_mean_k", p.outlier_mean_k);
  declare_parameter("outlier_std_mul", p.outlier_std_mul);
  declare_parameter("grid_resolution", p.grid_resolution);
  declare_parameter("ground_height", p.ground_height);
  declare_parameter("slope_tolerance", p.slope_tolerance);
  declare_parameter("max_height", p.max_height);
  declare_parameter("min_height", p.min_height);
  declare_parameter("suspend_gap", p.suspend_gap);
  declare_parameter("ground_median_kernel", p.ground_median_kernel);
  declare_parameter("enable_inflate", p.enable_inflate);
  declare_parameter("inflate_radius", p.inflate_radius);
  declare_parameter("enable_tbb", p.enable_tbb);
  // 累计点云参数
  declare_parameter("enable_accumulate", enable_accumulate_);
  declare_parameter("accumulate_range", accumulate_range_);
  declare_parameter("accumulate_lifetime", accumulate_lifetime_);

  get_parameter("enable_outlier", p.enable_outlier);
  get_parameter("outlier_mean_k", p.outlier_mean_k);
  get_parameter("outlier_std_mul", p.outlier_std_mul);
  get_parameter("grid_resolution", p.grid_resolution);
  get_parameter("ground_height", p.ground_height);
  get_parameter("slope_tolerance", p.slope_tolerance);
  get_parameter("max_height", p.max_height);
  get_parameter("min_height", p.min_height);
  get_parameter("suspend_gap", p.suspend_gap);
  get_parameter("ground_median_kernel", p.ground_median_kernel);
  get_parameter("enable_inflate", p.enable_inflate);
  get_parameter("inflate_radius", p.inflate_radius);
  get_parameter("enable_tbb", p.enable_tbb);
  get_parameter("enable_accumulate", enable_accumulate_);
  get_parameter("accumulate_range", accumulate_range_);
  get_parameter("accumulate_lifetime", accumulate_lifetime_);
  core_.setParams(p);

  // 话题名参数（默认私有命名，可在 yaml 中改为真实话题）
  declare_parameter("input_cloud_topic", topic_input_cloud_);
  declare_parameter("obstacles_topic", topic_obstacles_);
  declare_parameter("ground_topic", topic_ground_);
  declare_parameter("inflated_topic", topic_inflated_);
  get_parameter("input_cloud_topic", topic_input_cloud_);
  get_parameter("obstacles_topic", topic_obstacles_);
  get_parameter("ground_topic", topic_ground_);
  get_parameter("inflated_topic", topic_inflated_);

  spdlog::info("[cloud_preprocess_node] 参数已加载: grid={}m height={}m "
               "accumulate={} r={}m life={}s",
               p.grid_resolution, p.ground_height,
               enable_accumulate_ ? "on" : "off", accumulate_range_,
               accumulate_lifetime_);
}

void CloudPreprocessNode::processAndPublish(
    const CloudPtr &cloud, const std_msgs::msg::Header &header) {
  // 选择处理对象：实时单帧 或 累计输入点云
  CloudPtr process_input(new CloudT(*cloud));
  if (enable_accumulate_) {
    accumulate(cloud, rclcpp::Time(header.stamp));
    pruneAccumulation(rclcpp::Time(header.stamp));
    process_input = getAccumulated();
  }

  CloudPtr obstacles(new CloudT);
  CloudPtr ground(new CloudT);
  pcl::PointCloud<pcl::PointXYZI>::Ptr obstacles_xyzi(
      new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr ground_xyzi(
      new pcl::PointCloud<pcl::PointXYZI>);

  core_.process(process_input, obstacles, ground);

  // 转换 obstacles：拷贝 xyz，强度赋 0
  obstacles_xyzi->resize(obstacles->size());
  for (size_t i = 0; i < obstacles->size(); ++i) {
    const auto &src = obstacles->at(i);
    auto &dst = obstacles_xyzi->at(i);
    dst.x = src.x;
    dst.y = src.y;
    dst.z = src.z;
    dst.intensity = 0.0f; // 设为 0
  }

  // 转换 ground：同理
  ground_xyzi->resize(ground->size());
  for (size_t i = 0; i < ground->size(); ++i) {
    const auto &src = ground->at(i);
    auto &dst = ground_xyzi->at(i);
    dst.x = src.x;
    dst.y = src.y;
    dst.z = src.z;
    dst.intensity = 0.0f;
  }

  // 发布带强度的点云
  sensor_msgs::msg::PointCloud2 out_grd, out_obs_msg;
  pcl::toROSMsg(*ground_xyzi, out_grd);
  pcl::toROSMsg(*obstacles_xyzi, out_obs_msg);
  out_grd.header = header;
  out_obs_msg.header = header;
  pub_ground_->publish(out_grd);
  pub_obstacles_->publish(out_obs_msg);

  // 可选：膨胀点云地图（基于当前输出的障碍物）
  if (core_.params().enable_inflate) {
    CloudPtr inflated(new CloudT);
    core_.inflateCloud(obstacles, inflated);
    sensor_msgs::msg::PointCloud2 out_inf;
    pcl::toROSMsg(*inflated, out_inf);
    out_inf.header = header;
    pub_inflated_->publish(out_inf);
  }
}

void CloudPreprocessNode::cloudCallback(
    const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
  CloudPtr cloud(new CloudT);
  pcl::fromROSMsg(*msg, *cloud);
  processAndPublish(cloud, msg->header);
}

void CloudPreprocessNode::accumulate(const CloudPtr &input,
                                     const rclcpp::Time &stamp) {
  for (const auto &p : input->points) {
    acc_pts_.push_back(p);
    acc_stamps_.push_back(stamp);
  }
}

void CloudPreprocessNode::pruneAccumulation(const rclcpp::Time &now) {
  std::vector<PointT> kept;
  std::vector<rclcpp::Time> kept_t;
  kept.reserve(acc_pts_.size());
  kept_t.reserve(acc_pts_.size());
  const double life = accumulate_lifetime_;
  for (size_t i = 0; i < acc_pts_.size(); ++i) {
    const double age = (now - acc_stamps_[i]).seconds();
    if (age > life)
      continue; // 时间衰减
    kept.push_back(acc_pts_[i]);
    kept_t.push_back(acc_stamps_[i]);
  }
  acc_pts_.swap(kept);
  acc_stamps_.swap(kept_t);
}

CloudPtr CloudPreprocessNode::getAccumulated() const {
  CloudPtr out(new CloudT);
  out->reserve(acc_pts_.size());
  for (const auto &p : acc_pts_)
    out->points.push_back(p);
  out->width = static_cast<uint32_t>(out->size());
  out->height = 1;
  return out;
}

} // namespace cloud_preprocess

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<cloud_preprocess::CloudPreprocessNode>());
  rclcpp::shutdown();
  return 0;
}
