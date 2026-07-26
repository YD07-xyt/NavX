#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include"sensor_msgs/msg/point_cloud2.hpp"
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <chrono>
#include <memory>
#include <string>

class TfSubscriberToOdometryNode : public rclcpp::Node
{
public:
    TfSubscriberToOdometryNode()
    : rclcpp::Node("fake_odom_pub_sim")
    {
        // 声明参数
        this->declare_parameter<std::string>("target_frame", "base_link");
        this->declare_parameter<std::string>("source_frame", "world");
        this->declare_parameter<double>("publish_frequency", 10.0);
        
        target_frame_ = this->get_parameter("target_frame").as_string();
        source_frame_ = this->get_parameter("source_frame").as_string();
        double publish_freq = this->get_parameter("publish_frequency").as_double();
        
        // 创建TF缓冲区（自动订阅TF话题）
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        
        // 创建里程计发布者
        odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>(
            "/fake_odom", 10);
        
        // 创建定时器，定期查询TF并发布odom
        timer_ = this->create_wall_timer(
            std::chrono::duration<double>(1.0 / publish_freq),
            std::bind(&TfSubscriberToOdometryNode::publish_odometry, this));
            
        RCLCPP_INFO(this->get_logger(), 
                   "TF Subscriber to Odometry Node Started");
        RCLCPP_INFO(this->get_logger(), 
                   "Looking for transform: %s -> %s",
                   source_frame_.c_str(), target_frame_.c_str());
                           // 声明参数：输入/输出话题名
        this->declare_parameter<std::string>("input_topic", "/livox/lidar/pointcloud");
        this->declare_parameter<std::string>("output_topic", "/output_cloud");

        std::string in_topic = this->get_parameter("input_topic").as_string();
        std::string out_topic = this->get_parameter("output_topic").as_string();

        // 订阅输入点云
        pcl_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            in_topic, 10,
            std::bind(&TfSubscriberToOdometryNode::cloud_callback, this, std::placeholders::_1));

        // 发布输出点云
        pcl_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(out_topic, 10);

        RCLCPP_INFO(this->get_logger(),
            "RGB to Intensity converter ready. Sub: %s, Pub: %s",
            in_topic.c_str(), out_topic.c_str());
    }

private:
    void publish_odometry()
    {
        geometry_msgs::msg::TransformStamped transform_stamped;
        
        try {
            // 从TF缓冲区获取最新的变换（实际上是在查询订阅到的TF数据）
            transform_stamped = tf_buffer_->lookupTransform(
                source_frame_, target_frame_,
                tf2::TimePointZero);  // 最新可用的变换
                
        } catch (const tf2::TransformException & ex) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), 
                               *this->get_clock(), 5000,  // 5秒打印一次
                               "Transform not available: %s", ex.what());
            return;
        }
        
        // 创建并填充里程计消息
        auto odom_msg = nav_msgs::msg::Odometry();
        
        // 设置时间戳和坐标系
        odom_msg.header.stamp = this->get_clock()->now();
        odom_msg.header.frame_id = source_frame_;  // world frame
        odom_msg.child_frame_id = target_frame_;  // base_link
        
        // 设置位置
        odom_msg.pose.pose.position.x = transform_stamped.transform.translation.x;
        odom_msg.pose.pose.position.y = transform_stamped.transform.translation.y;
        odom_msg.pose.pose.position.z = transform_stamped.transform.translation.z;
        
        // 设置方向（四元数）
        odom_msg.pose.pose.orientation = transform_stamped.transform.rotation;
        
        // 设置协方差矩阵（可根据需要调整）
        // 位置协方差
        odom_msg.pose.covariance = {
            0.01, 0, 0, 0, 0, 0,
            0, 0.01, 0, 0, 0, 0,
            0, 0, 0.01, 0, 0, 0,
            0, 0, 0, 0.01, 0, 0,
            0, 0, 0, 0, 0.01, 0,
            0, 0, 0, 0, 0, 0.01
        };
        
        // 速度部分（设置为0或根据需要计算）
        odom_msg.twist.twist.linear.x = 0.0;
        odom_msg.twist.twist.linear.y = 0.0;
        odom_msg.twist.twist.linear.z = 0.0;
        odom_msg.twist.twist.angular.x = 0.0;
        odom_msg.twist.twist.angular.y = 0.0;
        odom_msg.twist.twist.angular.z = 0.0;
        
        // 速度协方差
        odom_msg.twist.covariance = {
            0.1, 0, 0, 0, 0, 0,
            0, 0.1, 0, 0, 0, 0,
            0, 0, 0.1, 0, 0, 0,
            0, 0, 0, 0.1, 0, 0,
            0, 0, 0, 0, 0.1, 0,
            0, 0, 0, 0, 0, 0.1
        };
        
        // 发布消息
        odom_publisher_->publish(odom_msg);
        
        RCLCPP_DEBUG(this->get_logger(), 
                    "Published odometry: x=%.3f, y=%.3f, z=%.3f",
                    transform_stamped.transform.translation.x,
                    transform_stamped.transform.translation.y,
                    transform_stamped.transform.translation.z);
    }
      void cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr input)
    {
        // 检查输入是否包含 xyz 和 rgb 字段
        bool has_xyz = false, has_rgb = false;
        for (const auto & field : input->fields) {
            if (field.name == "x") has_xyz = true;
            if (field.name == "rgb") has_rgb = true;
        }
        if (!has_xyz || !has_rgb) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                "Input cloud must contain 'xyz' and 'rgb' fields, skipping.");
            return;
        }

        // 创建输出点云，手动设置字段：x, y, z, intensity
        auto output = std::make_unique<sensor_msgs::msg::PointCloud2>();
        output->header = input->header;   // 保留原时间戳和坐标系

        output->fields.resize(4);
        output->fields[0].name = "x";
        output->fields[0].offset = 0;
        output->fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
        output->fields[0].count = 1;

        output->fields[1].name = "y";
        output->fields[1].offset = 4;
        output->fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
        output->fields[1].count = 1;

        output->fields[2].name = "z";
        output->fields[2].offset = 8;
        output->fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
        output->fields[2].count = 1;

        output->fields[3].name = "intensity";
        output->fields[3].offset = 12;
        output->fields[3].datatype = sensor_msgs::msg::PointField::FLOAT32;
        output->fields[3].count = 1;

        output->point_step = 16;                            // 4 * float
        output->data.resize(input->width * input->height * output->point_step);
        output->width = input->width;
        output->height = input->height;
        output->is_dense = input->is_dense;

        // 输入迭代器
        sensor_msgs::PointCloud2ConstIterator<float> in_x(*input, "x");
        sensor_msgs::PointCloud2ConstIterator<float> in_y(*input, "y");
        sensor_msgs::PointCloud2ConstIterator<float> in_z(*input, "z");
        sensor_msgs::PointCloud2ConstIterator<float> in_rgb(*input, "rgb");

        // 输出迭代器
        sensor_msgs::PointCloud2Iterator<float> out_x(*output, "x");
        sensor_msgs::PointCloud2Iterator<float> out_y(*output, "y");
        sensor_msgs::PointCloud2Iterator<float> out_z(*output, "z");
        sensor_msgs::PointCloud2Iterator<float> out_intensity(*output, "intensity");

        // 逐点拷贝并转换
        for (size_t i = 0; i < input->width * input->height; ++i) {
            *out_x = *in_x;
            *out_y = *in_y;
            *out_z = *in_z;
            *out_intensity = *in_rgb;   // rgb 的 float 值直接作为强度

            ++in_x; ++in_y; ++in_z; ++in_rgb;
            ++out_x; ++out_y; ++out_z; ++out_intensity;
        }

        pcl_pub_->publish(std::move(output));
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pcl_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pcl_pub_;
    // 成员变量
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    std::string target_frame_;
    std::string source_frame_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TfSubscriberToOdometryNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}