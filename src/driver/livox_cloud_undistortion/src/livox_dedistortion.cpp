
#include <condition_variable>
#include <csignal>
#include <deque>
#include <memory>
#include <mutex>
#include <rclcpp/executors.hpp>
#include <rclcpp/node.hpp>
#include <sensor_msgs/msg/detail/imu__struct.hpp>
#include <sensor_msgs/msg/detail/point_cloud2__struct.hpp>
#include <spdlog/spdlog.h>
#include <thread>

#include "livox_dedistortion_pkg/data_process.h"

/// *************Config data
std::string topic_pcl = "/livox/lidar_pc2";
std::string topic_imu = "/livox/imu";
/// *************

/// To notify new data
std::mutex mtx_buffer;
std::condition_variable sig_buffer;
bool b_exit = false;
bool b_reset = false;

/// Buffers for measurements
double last_timestamp_lidar = -1;
std::deque<sensor_msgs::msg::PointCloud2::SharedPtr> lidar_buffer;
double last_timestamp_imu = -1;
std::deque<sensor_msgs::msg::Imu::SharedPtr> imu_buffer;

void SigHandle(int sig) {
  b_exit = true;

  spdlog::warn("catch sig {}", sig);
  sig_buffer.notify_all();
}

void pointcloud_cbk(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
  const double timestamp = msg->header.stamp.sec;
  // ROS_DEBUG("get point cloud at time: %.6f", timestamp);
  mtx_buffer.lock();
  if (timestamp < last_timestamp_lidar) {
    spdlog::error("lidar loop back, clear buffer");
    lidar_buffer.clear();
  }
  last_timestamp_lidar = timestamp;
  lidar_buffer.push_back(msg);
  std::cout << "received point size: "
            << float(msg->data.size()) / float(msg->point_step) << "\n";
  mtx_buffer.unlock();

  sig_buffer.notify_all();
}

void imu_cbk(const sensor_msgs::msg::Imu::SharedPtr msg_in) {
  sensor_msgs::msg::Imu::SharedPtr msg(new sensor_msgs::msg::Imu(*msg_in));

  double timestamp = msg->header.stamp.sec;
  // ROS_DEBUG("get imu at time: %.6f", timestamp);

  mtx_buffer.lock();

  if (timestamp < last_timestamp_imu) {
    spdlog::error("imu loop back, clear buffer");
    imu_buffer.clear();
    b_reset = true;
  }
  last_timestamp_imu = timestamp;

  imu_buffer.push_back(msg);

  mtx_buffer.unlock();
  sig_buffer.notify_all();
}

bool SyncMeasure(MeasureGroup &measgroup) {
  if (lidar_buffer.empty() || imu_buffer.empty()) {
    /// Note: this will happen
    return false;
  }
  if (imu_buffer.front()->header.stamp.sec >
      lidar_buffer.back()->header.stamp.sec) {
    lidar_buffer.clear();
    spdlog::error("clear lidar buffer, only happen at the beginning");
    return false;
  }

  if (imu_buffer.back()->header.stamp.sec <
      lidar_buffer.front()->header.stamp.sec) {
    return false;
  }

  /// Add lidar data, and pop from buffer
  measgroup.lidar = lidar_buffer.front();
  lidar_buffer.pop_front();
  double lidar_time = measgroup.lidar->header.stamp.sec;

  /// Add imu data, and pop from buffer
  measgroup.imu.clear();
  int imu_cnt = 0;
  for (const auto &imu : imu_buffer) {
    double imu_time = imu->header.stamp.sec;
    if (imu_time <= lidar_time) {
      measgroup.imu.push_back(imu);
      imu_cnt++;
    }
  }
  for (int i = 0; i < imu_cnt; ++i) {
    imu_buffer.pop_front();
  }
  // ROS_DEBUG("add %d imu msg", imu_cnt);

  return true;
}

void ProcessLoop(std::shared_ptr<ImuProcess> p_imu) {

  spdlog::info("Start ProcessLoop");
  rclcpp::Rate r(1000);
  while (rclcpp::ok()) {
    MeasureGroup meas;
    std::unique_lock<std::mutex> lk(mtx_buffer);
    sig_buffer.wait(lk,
                    [&meas]() -> bool { return SyncMeasure(meas) || b_exit; });
    lk.unlock();

    if (b_exit) {
      spdlog::info("b_exit=true, exit");
      break;
    }

    if (b_reset) {
      spdlog::warn("reset when rosbag play back");
      p_imu->Reset();
      b_reset = false;
      continue;
    }
    p_imu->Process(meas);
    r.sleep();
  }
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto nh = std::make_shared<rclcpp::Node>("livox_dedistortion_node");
  signal(SIGINT, SigHandle);

  auto sub_pcl = nh->create_subscription<sensor_msgs::msg::PointCloud2>(
      topic_pcl, 100, pointcloud_cbk);
  auto sub_imu =
      nh->create_subscription<sensor_msgs::msg::Imu>(topic_imu, 1000, imu_cbk);

  std::shared_ptr<ImuProcess> p_imu(new ImuProcess());

  std::vector<double> vec;

  // 先声明参数（类型需匹配 vec 的类型，例如 std::vector<double>）
  nh->declare_parameter<std::vector<double>>("ExtIL", std::vector<double>());
  // 再获取参数值到 vec
  nh->get_parameter("ExtIL", vec);
  if (nh->get_parameter("ExtIL", vec)) {
    Eigen::Quaternion<double> q_il;
    Eigen::Vector3d t_il;
    q_il.w() = vec[0];
    q_il.x() = vec[1];
    q_il.y() = vec[2];
    q_il.z() = vec[3];
    t_il << vec[4], vec[5], vec[6];
    p_imu->set_T_i_l(q_il, t_il);

    spdlog::info("Extrinsic Parameter RESET ...");
  }

  /// for debug
  p_imu->nh = nh;

  std::thread th_proc(ProcessLoop, p_imu);

  // ros::spin();
  rclcpp::Rate r(1000);
  while (rclcpp::ok()) {
    if (b_exit)
      break;
    rclcpp::spin_some(nh);
    r.sleep();
  }

  spdlog::info("Wait for process loop exit");
  if (th_proc.joinable())
    th_proc.join();

  return 0;
}
