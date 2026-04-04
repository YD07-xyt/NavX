#include "st_opt_ros2.h"
#include <memory>
#include <nav_msgs/msg/path.hpp>

namespace st {

void StPlanner::configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr& parent,
    std::string name, 
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
    RCLCPP_INFO(logger_, "StPlanner::configure START");
    
    if (!costmap_ros) {
        RCLCPP_ERROR(logger_, "costmap_ros is null!");
        return;
    }
    
    costmap_ros_ = costmap_ros;
    costmap_ = costmap_ros_->getCostmap();
    
    if (!costmap_) {
        RCLCPP_ERROR(logger_, "Failed to get costmap!");
        return;
    }
    
    // 获取地图原点信息
    map_origin_x_ = costmap_->getOriginX();
    map_origin_y_ = costmap_->getOriginY();
    
    esdf_map = std::make_shared<grid_map::GridMap>();
    
    RCLCPP_INFO(logger_, "StPlanner::configure SUCCESS - Map origin: (%.2f, %.2f)", 
                map_origin_x_, map_origin_y_);
}

// 世界坐标转地图坐标（米）
Eigen::Vector2d StPlanner::worldToMap(const Eigen::Vector2d& world_pt) {
    return Eigen::Vector2d(
        world_pt.x() - map_origin_x_,
        world_pt.y() - map_origin_y_
    );
}

// 检查并修复起点/终点
bool StPlanner::ensureValidPoint(Eigen::Vector2d& point) {
    unsigned int mx, my;
    if (!costmap_->worldToMap(point.x(), point.y(), mx, my)) {
        RCLCPP_WARN(logger_, "Point (%.2f, %.2f) is out of map bounds!", point.x(), point.y());
        return false;
    }
    
    unsigned char cost = costmap_->getCost(mx, my);
    if (cost >= nav2_costmap_2d::LETHAL_OBSTACLE) {
        RCLCPP_WARN(logger_, "Point (%.2f, %.2f) is in obstacle! Searching for nearby free point...", 
                    point.x(), point.y());
        
        // 搜索附近空闲点
        unsigned int size_x = costmap_->getSizeInCellsX();
        unsigned int size_y = costmap_->getSizeInCellsY();
        
        for (int radius = 1; radius <= 20; radius++) {
            for (int dx = -radius; dx <= radius; dx++) {
                for (int dy = -radius; dy <= radius; dy++) {
                    int nx = mx + dx;
                    int ny = my + dy;
                    if (nx >= 0 && nx < (int)size_x && ny >= 0 && ny < (int)size_y) {
                        unsigned char test_cost = costmap_->getCost(nx, ny);
                        if (test_cost < nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE) {
                            // 找到空闲点
                            double wx, wy;
                            costmap_->mapToWorld(nx, ny, wx, wy);
                            point.x() = wx;
                            point.y() = wy;
                            RCLCPP_WARN(logger_, "Moved point to free location: (%.2f, %.2f)", 
                                        point.x(), point.y());
                            return true;
                        }
                    }
                }
            }
        }
        return false;
    }
    return true;
}

nav_msgs::msg::Path StPlanner::createPlan(
    const geometry_msgs::msg::PoseStamped& start,
    const geometry_msgs::msg::PoseStamped& goal)
{
    RCLCPP_ERROR(logger_, "=================== ST_PLANNER CREATE PLAN CALLED ===================");
    
    nav_msgs::msg::Path nav2_path;
    
    // 检查指针有效性
    if (costmap_ == nullptr || esdf_map == nullptr) {
        RCLCPP_ERROR(logger_, "Costmap or ESDF map not available");
        return nav2_path;
    }
    
    // 获取地图参数
    double resolution = costmap_->getResolution();
    unsigned int size_x = costmap_->getSizeInCellsX();
    unsigned int size_y = costmap_->getSizeInCellsY();
    
    RCLCPP_INFO(logger_, "Costmap size: %u x %u, resolution: %.3f, origin: (%.2f, %.2f)", 
                size_x, size_y, resolution, map_origin_x_, map_origin_y_);
    
    // 转换起点和终点为世界坐标（已经是世界坐标，但确保在costmap范围内）
    Eigen::Vector2d start_world(start.pose.position.x, start.pose.position.y);
    Eigen::Vector2d goal_world(goal.pose.position.x, goal.pose.position.y);
    
    // ✅ 关键修复：检查并修复起点和终点
    if (!ensureValidPoint(start_world)) {
        RCLCPP_ERROR(logger_, "Cannot find valid start point!");
        return nav2_path;
    }
    
    if (!ensureValidPoint(goal_world)) {
        RCLCPP_ERROR(logger_, "Cannot find valid goal point!");
        // 尝试扩大搜索范围
        RCLCPP_WARN(logger_, "Goal point invalid, creating simple straight line path as fallback");
    }
    
    RCLCPP_INFO(logger_, "Valid Start: (%.2f, %.2f), Goal: (%.2f, %.2f)", 
                start_world.x(), start_world.y(), goal_world.x(), goal_world.y());
    
    // 转换为地图坐标系（相对于地图原点）
    Eigen::Vector2d start_map = worldToMap(start_world);
    Eigen::Vector2d goal_map = worldToMap(goal_world);
    
    RCLCPP_INFO(logger_, "Map coordinates - Start: (%.2f, %.2f), Goal: (%.2f, %.2f)", 
                start_map.x(), start_map.y(), goal_map.x(), goal_map.y());
    
    // 创建占据栅格地图（用于A*）
    grid_map::RowMatrixXi occupancy(size_y, size_x);
    unsigned char* costmap_data = costmap_->getCharMap();
    
    int obstacle_count = 0;
    for (unsigned int my = 0; my < size_y; ++my) {
        for (unsigned int mx = 0; mx < size_x; ++mx) {
            unsigned int index = my * size_x + mx;
            unsigned char cost = costmap_data[index];
            
            // 致命障碍物视为障碍物
        //if (cost>220&&cost<=nav2_costmap_2d::LETHAL_OBSTACLE) {
        if (cost==nav2_costmap_2d::LETHAL_OBSTACLE) {
            occupancy(mx, my) = 1;
                obstacle_count++;
            } else {
                occupancy(mx, my) = 0;  // 包括未知区域
            }
        }
    }
    
    RCLCPP_INFO(logger_, "Obstacle ratio: %.1f%% (%d/%u)", 
                100.0 * obstacle_count / (size_x * size_y), 
                obstacle_count, size_x * size_y);
    
    // 设置 ESDF 地图
    double map_size_x = size_x * resolution;
    double map_size_y = size_y * resolution;
    esdf_map->init(map_size_x, map_size_y, resolution);
    esdf_map->setMap(occupancy);
    
    // A* 路径规划（使用地图坐标系坐标）
    path_planning::AStar astar(*esdf_map, 0.2);
    auto astar_traj = astar.planWithPostProcessing(start_map, goal_map, config_.timeout_ms);
    
    if (astar_traj.optimized_path.empty()) {
        RCLCPP_ERROR(logger_, "A* planning failed! No path found.");
        return nav2_path;
    }
    
    RCLCPP_INFO(logger_, "A* found path with %zu points, length: %.2f", 
                astar_traj.optimized_path.size(), astar_traj.total_length);
    
    // 轨迹优化
    TrajOpt::TrajectoryParams params;
    params.piece_len = 0.1;
    params.total_time = astar_traj.total_time;
    params.total_len = astar_traj.total_length;
    
    TrajOpt::TrajectoryOptimizer optimizer(
        esdf_map,
        astar_traj.optimized_path,
        params
    );
    
    nav2_path.header.frame_id = costmap_ros_->getGlobalFrameID();
    nav2_path.header.stamp = rclcpp::Clock().now();
    
    if (!optimizer.plan()) {
        RCLCPP_WARN(logger_, "Trajectory optimization failed, using A* path");
        // 使用 A* 路径（需要转换回世界坐标）
        for (const auto& wp_map : astar_traj.optimized_path) {
            geometry_msgs::msg::PoseStamped pose;
            pose.header = nav2_path.header;
            // 将地图坐标转换回世界坐标
            pose.pose.position.x = wp_map.x() + map_origin_x_;
            pose.pose.position.y = wp_map.y() + map_origin_y_;
            pose.pose.position.z = 0.0;
            pose.pose.orientation.w = 1.0;
            nav2_path.poses.push_back(pose);
        }
        return nav2_path;
    }
    
    // 采样优化后的轨迹（转换回世界坐标）
    auto sampled_traj = optimizer.sampleTrajectory(0.1);
    RCLCPP_INFO(logger_, "Optimized trajectory has %zu points", sampled_traj.size());
    
    for (const auto& wp_map : sampled_traj) {
        geometry_msgs::msg::PoseStamped pose;
        pose.header = nav2_path.header;
        // 将地图坐标转换回世界坐标
        pose.pose.position.x = wp_map.x() + map_origin_x_;
        pose.pose.position.y = wp_map.y() + map_origin_y_;
        pose.pose.position.z = 0.0;
        pose.pose.orientation.w = 1.0;
        nav2_path.poses.push_back(pose);
    }
    
    RCLCPP_INFO(logger_, "createPlan SUCCESS, path size: %zu", nav2_path.poses.size());
    return nav2_path;
}

} // namespace st

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(st::StPlanner, nav2_core::GlobalPlanner)