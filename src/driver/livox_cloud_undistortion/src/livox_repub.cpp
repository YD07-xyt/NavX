#include "livox_ros_driver2/msg/custom_msg.hpp"
#include <memory>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <vector>

typedef pcl::PointXYZINormal PointType;

rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_pcl_out1;  // 只保留用到的
uint64_t TO_MERGE_CNT = 1;
std::vector<livox_ros_driver2::msg::CustomMsg::ConstSharedPtr> livox_data;  // 推荐存 ConstSharedPtr

void LivoxMsgCbk1(const livox_ros_driver2::msg::CustomMsg::ConstSharedPtr &livox_msg_in) {
    livox_data.push_back(livox_msg_in);
    if (livox_data.size() < TO_MERGE_CNT)
        return;

    pcl::PointCloud<PointType> pcl_in;

    for (size_t j = 0; j < livox_data.size(); j++) {
        auto &livox_msg = livox_data[j];
        auto time_end = livox_msg->points.back().offset_time;
        for (unsigned int i = 0; i < livox_msg->point_num; ++i) {
            PointType pt;
            pt.x = livox_msg->points[i].x;
            pt.y = livox_msg->points[i].y;
            pt.z = livox_msg->points[i].z;
            float s = livox_msg->points[i].offset_time / (float)time_end;
            pt.intensity = livox_msg->points[i].line +
                           s * 0.1;
            pt.curvature = livox_msg->points[i].reflectivity * 0.1;
            pcl_in.push_back(pt);
        }
    }

    uint64_t timebase_ns = livox_data[0]->timebase;
    rclcpp::Time timestamp(timebase_ns, RCL_ROS_TIME);

    sensor_msgs::msg::PointCloud2 pcl_ros_msg;
    pcl::toROSMsg(pcl_in, pcl_ros_msg);  // 推荐
    pcl_ros_msg.header.stamp = timestamp;
    pcl_ros_msg.header.frame_id = "livox";  // 去掉前导 /

    pub_pcl_out1->publish(pcl_ros_msg);  // 发布转换后的点云
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto nh = std::make_shared<rclcpp::Node>("livox_repub_node");
    RCLCPP_INFO(nh->get_logger(), "start livox_repub");

    auto sub_livox_msg1 =
        nh->create_subscription<livox_ros_driver2::msg::CustomMsg>(
            "/livox/lidar", 100, LivoxMsgCbk1);

    pub_pcl_out1 = nh->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/livox/lidar_pc2", 100);

    rclcpp::spin(nh);
    rclcpp::shutdown();
    return 0;
}