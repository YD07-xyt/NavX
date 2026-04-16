#include "st_opt_ros2.h"
#include <memory>
#include <nav_msgs/msg/path.hpp>
#include <chrono>

namespace st {

void StPlanner::configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr& parent,
    std::string name, 
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
    RCLCPP_INFO(logger_, "StPlanner::configure START");
    node_ = parent.lock();
    plugin_name_ = name;
    
    // 声明轨迹参数组的每个成员
    auto param_namespace = plugin_name_ + ".trajectory";
    
    // double 类型参数
    node_->declare_parameter(param_namespace + ".rho_v", rclcpp::ParameterValue(10000.0));
    node_->declare_parameter(param_namespace + ".rho_collision", rclcpp::ParameterValue(100000.0));
    node_->declare_parameter(param_namespace + ".rho_T", rclcpp::ParameterValue(100.0));
    node_->declare_parameter(param_namespace + ".rho_energy", rclcpp::ParameterValue(100.0));
    node_->declare_parameter(param_namespace + ".max_v", rclcpp::ParameterValue(1.0));
    node_->declare_parameter(param_namespace + ".safe_threshold", rclcpp::ParameterValue(0.75));
    
    // int 类型参数
    node_->declare_parameter(param_namespace + ".int_K", rclcpp::ParameterValue(32));
    node_->declare_parameter(param_namespace + ".mem_size", rclcpp::ParameterValue(256));
    node_->declare_parameter(param_namespace + ".past", rclcpp::ParameterValue(3));
    node_->declare_parameter(param_namespace + ".max_iter", rclcpp::ParameterValue(10000));
    
    // double 类型（小数值）
    node_->declare_parameter(param_namespace + ".g_epsilon", rclcpp::ParameterValue(1e-6));
    node_->declare_parameter(param_namespace + ".min_step", rclcpp::ParameterValue(1e-32));
    node_->declare_parameter(param_namespace + ".delta", rclcpp::ParameterValue(1e-5));
    
    node_->declare_parameter(param_namespace + ".timeout_ms", rclcpp::ParameterValue(5000));
    node_->declare_parameter(param_namespace + ".min_obstacle_cost", rclcpp::ParameterValue(220));
    node_->declare_parameter(param_namespace + ".max_obstacle_cost", rclcpp::ParameterValue(254));
    node_->declare_parameter(param_namespace + ".max_radius_grid_num", rclcpp::ParameterValue(50));
    node_->declare_parameter(param_namespace + ".search_resolution", rclcpp::ParameterValue(0.2));
    
    // 读取参数到结构体
    TrajOpt::TrajectoryParams params;
    Config config;
    node_->get_parameter(param_namespace + ".max_radius_grid_num", config.max_radius_grid_num);
    node_->get_parameter(param_namespace + ".timeout_ms", config.timeout_ms);
    node_->get_parameter(param_namespace + ".min_obstacle_cost", config.min_obstacle_cost);
    node_->get_parameter(param_namespace + ".max_obstacle_cost", config.max_obstacle_cost);
    node_->get_parameter(param_namespace + ".search_resolution", config.search_resolution);
    node_->get_parameter(param_namespace + ".rho_v", params.rho_v);
    node_->get_parameter(param_namespace + ".rho_collision", params.rho_collision);
    node_->get_parameter(param_namespace + ".rho_T", params.rho_T);
    node_->get_parameter(param_namespace + ".rho_energy", params.rho_energy);
    node_->get_parameter(param_namespace + ".max_v", params.max_v);
    node_->get_parameter(param_namespace + ".safe_threshold", params.safe_threshold);
    node_->get_parameter(param_namespace + ".int_K", params.int_K);
    node_->get_parameter(param_namespace + ".mem_size", params.mem_size);
    node_->get_parameter(param_namespace + ".past", params.past);
    node_->get_parameter(param_namespace + ".max_iter", params.max_iter);
    node_->get_parameter(param_namespace + ".g_epsilon", params.g_epsilon);
    node_->get_parameter(param_namespace + ".min_step", params.min_step);
    node_->get_parameter(param_namespace + ".delta", params.delta);
    
    params_ = params;
    config_ = config;
    
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
    
    // 获取地图参数
    map_origin_x_ = costmap_->getOriginX();
    map_origin_y_ = costmap_->getOriginY();
    resolution_ = costmap_->getResolution();
    size_x_ = costmap_->getSizeInCellsX();
    size_y_ = costmap_->getSizeInCellsY();
    
    esdf_map = std::make_shared<grid_map::GridMap>();
    
    RCLCPP_INFO(logger_, "StPlanner::configure SUCCESS - Map origin: (%.2f, %.2f), resolution: %.3f, size: %u x %u", 
                map_origin_x_, map_origin_y_, resolution_, size_x_, size_y_);
}

// 世界坐标 -> 地图坐标（米，相对于地图原点）
Eigen::Vector2d StPlanner::worldToMap(const Eigen::Vector2d& world_pt) {
    return Eigen::Vector2d(
        world_pt.x() - map_origin_x_,
        world_pt.y() - map_origin_y_
    );
}

// 地图坐标 -> 世界坐标
Eigen::Vector2d StPlanner::mapToWorld(const Eigen::Vector2d& map_pt) {
    return Eigen::Vector2d(
        map_pt.x() + map_origin_x_,
        map_pt.y() + map_origin_y_
    );
}

// 世界坐标 -> 网格索引
bool StPlanner::worldToGrid(const Eigen::Vector2d& world_pt, unsigned int& mx, unsigned int& my) {
    Eigen::Vector2d map_pt = worldToMap(world_pt);
    
    // 检查是否在边界内
    if (map_pt.x() < 0 || map_pt.x() > size_x_ * resolution_ ||
        map_pt.y() < 0 || map_pt.y() > size_y_ * resolution_) {
        return false;
    }
    
    mx = static_cast<unsigned int>(map_pt.x() / resolution_);
    my = static_cast<unsigned int>(map_pt.y() / resolution_);
    
    // 边界保护
    mx = std::min(mx, size_x_ - 1);
    my = std::min(my, size_y_ - 1);
    
    return true;
}

// 网格索引 -> 世界坐标
bool StPlanner::gridToWorld(unsigned int mx, unsigned int my, double& wx, double& wy) {
    if (mx >= size_x_ || my >= size_y_) {
        return false;
    }
    
    double map_x = (mx + 0.5) * resolution_;
    double map_y = (my + 0.5) * resolution_;
    wx = map_x + map_origin_x_;
    wy = map_y + map_origin_y_;
    return true;
}

// 检查网格是否有效
bool StPlanner::isValidGridCell(unsigned int mx, unsigned int my) {
    return (mx < size_x_ && my < size_y_);
}

// 检查网格是否空闲
bool StPlanner::isFreeCell(unsigned int mx, unsigned int my) {
    if (!isValidGridCell(mx, my)) return false;
    unsigned char cost = costmap_->getCost(mx, my);
    return (cost < config_.min_obstacle_cost);
}

// 查找最近的空闲网格
bool StPlanner::findNearestFreeCell(unsigned int mx, unsigned int my, 
                                     unsigned int& free_mx, unsigned int& free_my) {
    if (isFreeCell(mx, my)) {
        free_mx = mx;
        free_my = my;
        return true;
    }
    
    // 螺旋搜索
    for (int radius = 1; radius <= config_.max_radius_grid_num; radius++) {
        for (int dx = -radius; dx <= radius; dx++) {
            for (int dy = -radius; dy <= radius; dy++) {
                int nx = mx + dx;
                int ny = my + dy;
                if (nx >= 0 && nx < (int)size_x_ && ny >= 0 && ny < (int)size_y_) {
                    if (isFreeCell(nx, ny)) {
                        free_mx = nx;
                        free_my = ny;
                        return true;
                    }
                }
            }
        }
    }
    return false;
}

// 检查并修复起点/终点
bool StPlanner::ensureValidPoint(Eigen::Vector2d& point) {
    unsigned int mx, my;
    if (!worldToGrid(point, mx, my)) {
        RCLCPP_WARN(logger_, "Point (%.2f, %.2f) is out of map bounds!", 
                    point.x(), point.y());
        
        // 尝试将点限制在地图边界内
        Eigen::Vector2d map_pt = worldToMap(point);
        map_pt.x() = std::clamp(map_pt.x(), 0.0, size_x_ * resolution_);
        map_pt.y() = std::clamp(map_pt.y(), 0.0, size_y_ * resolution_);
        point = mapToWorld(map_pt);
        
        if (!worldToGrid(point, mx, my)) {
            return false;
        }
    }
    
    if (!isFreeCell(mx, my)) {
        RCLCPP_WARN(logger_, "Point (%.2f, %.2f) is in obstacle! Searching for nearby free point...", 
                    point.x(), point.y());
        
        unsigned int free_mx, free_my;
        if (findNearestFreeCell(mx, my, free_mx, free_my)) {
            double wx, wy;
            gridToWorld(free_mx, free_my, wx, wy);
            point.x() = wx;
            point.y() = wy;
            RCLCPP_WARN(logger_, "Moved point to free location: (%.2f, %.2f)", 
                        point.x(), point.y());
            return true;
        } else {
            RCLCPP_ERROR(logger_, "Cannot find free cell near (%.2f, %.2f)!", 
                        point.x(), point.y());
            return false;
        }
    }
    return true;
}

// 创建占据栅格地图
void StPlanner::createOccupancyGrid(grid_map::RowMatrixXi& occupancy) {
    occupancy.resize(size_y_, size_x_);
    unsigned char* costmap_data = costmap_->getCharMap();
    int obstacle_count = 0;
    
    for (unsigned int my = 0; my < size_y_; ++my) {
        for (unsigned int mx = 0; mx < size_x_; ++mx) {
            unsigned int index = my * size_x_ + mx;
            unsigned char cost = costmap_data[index];
            
            if (cost >= config_.min_obstacle_cost && cost <= config_.max_obstacle_cost) {
                occupancy(my, mx) = 1;  // 障碍物（注意：RowMatrixXi 是 (row, col) 即 (y, x)）
                obstacle_count++;
            } else {
                occupancy(my, mx) = 0;  // 自由空间
            }
        }
    }
    
    RCLCPP_INFO(logger_, "Obstacle ratio: %.1f%% (%d/%u)", 
                100.0 * obstacle_count / (size_x_ * size_y_), 
                obstacle_count, size_x_ * size_y_);
}

nav_msgs::msg::Path StPlanner::createPlan(
    const geometry_msgs::msg::PoseStamped& start,
    const geometry_msgs::msg::PoseStamped& goal)
{
    RCLCPP_INFO(logger_, "=================== ST_PLANNER CREATE PLAN CALLED ===================");
    auto start_time = std::chrono::steady_clock::now();
    nav_msgs::msg::Path nav2_path;
    
    // 检查指针有效性
    if (costmap_ == nullptr || esdf_map == nullptr) {
        RCLCPP_ERROR(logger_, "Costmap or ESDF map not available");
        return nav2_path;
    }
    
    RCLCPP_INFO(logger_, "Costmap size: %u x %u, resolution: %.3f, origin: (%.2f, %.2f)", 
                size_x_, size_y_, resolution_, map_origin_x_, map_origin_y_);
    
    // 转换起点和终点
    Eigen::Vector2d start_world(start.pose.position.x, start.pose.position.y);
    Eigen::Vector2d goal_world(goal.pose.position.x, goal.pose.position.y);
    
    RCLCPP_INFO(logger_, "Original Start: (%.2f, %.2f), Goal: (%.2f, %.2f)", 
                start_world.x(), start_world.y(), goal_world.x(), goal_world.y());
    
    // 检查并修复起点和终点
    if (!ensureValidPoint(start_world)) {
        RCLCPP_ERROR(logger_, "Cannot find valid start point!");
        return nav2_path;
    }
    
    if (!ensureValidPoint(goal_world)) {
        RCLCPP_ERROR(logger_, "Cannot find valid goal point!");
        return nav2_path;
    }
    
    RCLCPP_INFO(logger_, "Valid Start: (%.2f, %.2f), Goal: (%.2f, %.2f)", 
                start_world.x(), start_world.y(), goal_world.x(), goal_world.y());
    
    // 转换为地图坐标系（米，相对于地图原点）
    Eigen::Vector2d start_map = worldToMap(start_world);
    Eigen::Vector2d goal_map = worldToMap(goal_world);
    
    RCLCPP_INFO(logger_, "Map coordinates - Start: (%.2f, %.2f), Goal: (%.2f, %.2f)", 
                start_map.x(), start_map.y(), goal_map.x(), goal_map.y());
    
    // 检查地图坐标是否有效
    if (start_map.x() < 0 || start_map.x() > size_x_ * resolution_ ||
        start_map.y() < 0 || start_map.y() > size_y_ * resolution_) {
        RCLCPP_ERROR(logger_, "Start map coordinates out of bounds!");
        return nav2_path;
    }
    
    if (goal_map.x() < 0 || goal_map.x() > size_x_ * resolution_ ||
        goal_map.y() < 0 || goal_map.y() > size_y_ * resolution_) {
        RCLCPP_ERROR(logger_, "Goal map coordinates out of bounds!");
        return nav2_path;
    }
    
    // 创建占据栅格地图
    grid_map::RowMatrixXi occupancy;
    createOccupancyGrid(occupancy);
    
    // 设置 ESDF 地图
    double map_size_x = size_x_ * resolution_;
    double map_size_y = size_y_ * resolution_;
    esdf_map->init(map_size_x, map_size_y, resolution_);
    esdf_map->setMap(occupancy);
    
    // A* 路径规划（使用地图坐标系坐标，单位：米）
    path_planning::AStar astar(*esdf_map, config_.search_resolution);
    auto astar_traj = astar.planWithPostProcessing(start_map, goal_map, config_.timeout_ms);
    
    if (astar_traj.optimized_path.empty()) {
        RCLCPP_ERROR(logger_, "A* planning failed! No path found.");
        return nav2_path;
    }
    
    RCLCPP_INFO(logger_, "A* found path with %zu points, length: %.2f, time: %.2f", 
                astar_traj.optimized_path.size(), astar_traj.total_length, astar_traj.total_time);
    
    // 轨迹优化
    params_.piece_len = astar_traj.total_length / astar_traj.total_time;
    params_.total_time = astar_traj.total_time;
    params_.total_len = astar_traj.total_length;
    
    TrajOpt::TrajectoryOptimizer optimizer(
        esdf_map,
        astar_traj.optimized_path,
        params_
    );
    
    nav2_path.header.frame_id = costmap_ros_->getGlobalFrameID();
    nav2_path.header.stamp = rclcpp::Clock().now();
    
    auto start_time_opt = std::chrono::steady_clock::now();
    
    if (!optimizer.plan()) {
        RCLCPP_WARN(logger_, "Trajectory optimization failed, using A* path");
        // 使用 A* 路径（转换回世界坐标）
        for (const auto& wp_map : astar_traj.optimized_path) {
            geometry_msgs::msg::PoseStamped pose;
            pose.header = nav2_path.header;
            Eigen::Vector2d wp_world = mapToWorld(wp_map);
            pose.pose.position.x = wp_world.x();
            pose.pose.position.y = wp_world.y();
            pose.pose.position.z = 0.0;
            pose.pose.orientation.w = 1.0;
            nav2_path.poses.push_back(pose);
        }
        
        auto end_time = std::chrono::steady_clock::now();
        auto timer = std::chrono::duration<double>(end_time - start_time).count();
        RCLCPP_INFO(logger_, "ST_Planner planning time (A* only): %.3f seconds", timer);
        RCLCPP_INFO(logger_, "createPlan SUCCESS (A* fallback), path size: %zu", nav2_path.poses.size());
        return nav2_path;
    }
    
    // 采样优化后的轨迹（转换回世界坐标）
    auto sampled_traj = optimizer.sampleTrajectory(0.1);
    RCLCPP_INFO(logger_, "Optimized trajectory has %zu points", sampled_traj.size());
    
    for (const auto& wp_map : sampled_traj) {
        geometry_msgs::msg::PoseStamped pose;
        pose.header = nav2_path.header;
        Eigen::Vector2d wp_world = mapToWorld(wp_map);
        pose.pose.position.x = wp_world.x();
        pose.pose.position.y = wp_world.y();
        pose.pose.position.z = 0.0;
        pose.pose.orientation.w = 1.0;
        nav2_path.poses.push_back(pose);
    }
    
    auto end_time = std::chrono::steady_clock::now();
    auto timer = std::chrono::duration<double>(end_time - start_time).count();
    auto timer_opt = std::chrono::duration<double>(end_time - start_time_opt).count();
    
    RCLCPP_INFO(logger_, "ST_Planner total planning time: %.3f seconds (optimization: %.3f)", timer, timer_opt);
    RCLCPP_INFO(logger_, "createPlan SUCCESS, optimized path size: %zu", nav2_path.poses.size());
    return nav2_path;
}

} // namespace st

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(st::StPlanner, nav2_core::GlobalPlanner)