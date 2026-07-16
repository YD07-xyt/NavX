#pragma once
#include <memory>
#include <nav2_costmap_2d/cost_values.hpp>
#include <nav2_costmap_2d/costmap_2d.hpp>
#include <nav2_costmap_2d/costmap_2d_ros.hpp>
#include "perception_tool/grid_map.hpp"
#include <nav2_core/global_planner.hpp>
#include <rclcpp/node.hpp>
#include "astar.hpp"
#include "traj_opt.hpp"

namespace st {

struct Config{
    int timeout_ms = 5000;
    int min_obstacle_cost = 220;
    int max_obstacle_cost = nav2_costmap_2d::LETHAL_OBSTACLE;
    int max_radius_grid_num = 50;
    double search_resolution = 0.2;
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
    TrajOpt::TrajectoryParams params_;
    std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node_;
    std::string plugin_name_;
    
    // 地图参数
    double map_origin_x_{0.0};
    double map_origin_y_{0.0};
    double resolution_{0.05};
    unsigned int size_x_{0};
    unsigned int size_y_{0};
    
    // 坐标转换函数
    Eigen::Vector2d worldToMap(const Eigen::Vector2d& world_pt);
    Eigen::Vector2d mapToWorld(const Eigen::Vector2d& map_pt);
    bool worldToGrid(const Eigen::Vector2d& world_pt, unsigned int& mx, unsigned int& my);
    bool gridToWorld(unsigned int mx, unsigned int my, double& wx, double& wy);
    bool isValidGridCell(unsigned int mx, unsigned int my);
    bool isFreeCell(unsigned int mx, unsigned int my);
    
    // 辅助函数
    bool ensureValidPoint(Eigen::Vector2d& point);
    bool findNearestFreeCell(unsigned int mx, unsigned int my, unsigned int& free_mx, unsigned int& free_my);
    void createOccupancyGrid(grid_map::RowMatrixXi& occupancy);
};

} // namespace st