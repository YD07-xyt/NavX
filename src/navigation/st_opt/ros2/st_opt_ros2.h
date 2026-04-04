#pragma once
#include <memory>
#include <nav2_costmap_2d/cost_values.hpp>
#include <nav2_costmap_2d/costmap_2d.hpp>
#include <nav2_costmap_2d/costmap_2d_ros.hpp>
#include "perception_tool/grid_map.hpp"
#include <nav2_core/global_planner.hpp>
#include "astar.hpp"
#include "traj_opt.hpp"

namespace st {

struct Config{
    int timeout_ms = 5000;
    double min_obstacle_cost = nav2_costmap_2d::LETHAL_OBSTACLE;
};

class StPlanner: public nav2_core::GlobalPlanner {
public:
    StPlanner() = default;
    ~StPlanner() = default;
    
    void configure(
        const rclcpp_lifecycle::LifecycleNode::WeakPtr& parent,
        std::string name, 
        std::shared_ptr<tf2_ros::Buffer> tf,
        std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

    void cleanup() override { 
        RCLCPP_INFO(logger_, "StPlanner cleanup"); 
    }
    
    void activate() override { 
        RCLCPP_INFO(logger_, "StPlanner activate"); 
    }
    
    void deactivate() override { 
        RCLCPP_INFO(logger_, "StPlanner deactivate"); 
    }

    nav_msgs::msg::Path createPlan(
        const geometry_msgs::msg::PoseStamped& start,
        const geometry_msgs::msg::PoseStamped& goal) override;
        
private:
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
    nav2_costmap_2d::Costmap2D* costmap_{nullptr};
    std::shared_ptr<grid_map::GridMap> esdf_map;
    rclcpp::Logger logger_{rclcpp::get_logger("StPlanner")};
    Config config_;
    
    // 地图原点
    double map_origin_x_{0.0};
    double map_origin_y_{0.0};
    
    // 辅助函数
    Eigen::Vector2d worldToMap(const Eigen::Vector2d& world_pt);
    bool ensureValidPoint(Eigen::Vector2d& point);
};

} // namespace st