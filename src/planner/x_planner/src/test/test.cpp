#include "../../include/path_search/jps.h"
#include "../../include/visualization.hpp"
#include "../../include/map.h"
#include "../../include/trajectory_optimization/trajopt.h"

using namespace planner;
using namespace test;

// 像素坐标转世界坐标
std::vector<Point> pixelToWorld(const std::vector<Point>& pixel_path, double resolution = 0.05) {
    std::vector<Point> world_path;
    world_path.reserve(pixel_path.size());
    for (const auto& p : pixel_path) {
        world_path.emplace_back(p.x * resolution, p.y * resolution);
    }
    return world_path;
}

// 世界坐标转像素坐标（用于可视化）
std::vector<Point> worldToPixel(const std::vector<Point>& world_path, double resolution = 0.05) {
    std::vector<Point> pixel_path;
    pixel_path.reserve(world_path.size());
    for (const auto& p : world_path) {
        pixel_path.emplace_back(p.x / resolution, p.y / resolution);
    }
    return pixel_path;
}

int main() {
    SimpleOccupancyMap grid_map("/home/xyt/code/x_planner/include/png/rm2026.png");
     Map map(grid_map);
    
    Point start_pixel(11, 81);
    Point end_pixel(187, 49);
    
    std::cout << "\n像素格子导航开始" << std::endl;
    std::cout << "起点(像素): (" << start_pixel.x << ", " << start_pixel.y << ")" << std::endl;
    std::cout << "终点(像素): (" << end_pixel.x << ", " << end_pixel.y << ")" << std::endl;
    
    // JPS路径规划
    jps jps_planner(map);
    auto start_time = std::chrono::high_resolution_clock::now();

    auto path_opt = jps_planner.search(start_pixel, end_pixel);
    
    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = duration_cast<std::chrono::microseconds>(end_time - start_time);
    
    std::cout << "JPS耗时：" << duration.count()/1000 << " ms" << std::endl;
    
    if (!path_opt.has_value()) {
        std::cerr << "路径规划失败!" << std::endl;
        return -1;
    }
    
    const auto& pixel_path = path_opt.value();
    std::cout << "\n路径规划成功!" << std::endl;
    std::cout << "原始路径长度: " << pixel_path.size() << " 个像素点" << std::endl;
    
    // 坐标转换（像素 -> 世界坐标）
    double resolution = 0.05;
    std::vector<Point> world_path;
    world_path.reserve(pixel_path.size());
    for (const auto& p : pixel_path) {
        world_path.emplace_back(p.x * resolution, p.y * resolution);
    }
    
    std::cout << "\n=== MINCO轨迹优化 ===" << std::endl;
    std::cout << "世界坐标路径点数量: " << world_path.size() << std::endl;
    std::cout << "起点(世界): (" << world_path.front().x << ", " << world_path.front().y << ")" << std::endl;
    std::cout << "终点(世界): (" << world_path.back().x << ", " << world_path.back().y << ")" << std::endl;
     auto start_time1 = std::chrono::high_resolution_clock::now();
    // MINCO优化（降采样步长设为10，减少到约23个点）
    // //TarjOpt opt(world_path,map);
    // if (!opt.optimization(10)) {  // 降采样步长10
    //     std::cerr << "MINCO优化失败!" << std::endl;
    //     return -1;
    // }
    
    // auto opt_world_path = opt.get_traj();
    // if (!opt_world_path.has_value()) {
    //     std::cerr << "无法获取优化轨迹!" << std::endl;
    //     return -1;
    // }
    // auto end_time1 = std::chrono::high_resolution_clock::now();
    // auto duration1 = duration_cast<std::chrono::microseconds>(end_time - start_time);
    // std::cout << "minco耗时:" << duration.count()/1000 << " ms" << std::endl;
    
    // // 转换回像素坐标
    // std::vector<Point> opt_pixel_path;
    // opt_pixel_path.reserve(opt_world_path->size());
    // for (const auto& p : *opt_world_path) {
    //     opt_pixel_path.emplace_back(p.x / resolution, p.y / resolution);
    // }
    
    // // 计算路径长度
    // double orig_len = 0, opt_len = 0;
    // for (size_t i = 0; i < pixel_path.size() - 1; i++) {
    //     orig_len += std::hypot(pixel_path[i+1].x - pixel_path[i].x,
    //                            pixel_path[i+1].y - pixel_path[i].y);
    // }
    // for (size_t i = 0; i < opt_pixel_path.size() - 1; i++) {
    //     opt_len += std::hypot(opt_pixel_path[i+1].x - opt_pixel_path[i].x,
    //                           opt_pixel_path[i+1].y - opt_pixel_path[i].y);
    // }
    
    // std::cout << "\n路径长度对比:" << std::endl;
    // std::cout << "  原始JPS路径: " << orig_len << " 像素" << std::endl;
    // std::cout << "  MINCO优化: " << opt_len << " 像素 (" 
    //           << ((opt_len - orig_len) / orig_len * 100) << "%)" << std::endl;
    
    // // 可视化
    // grid_map.drawPath(pixel_path, cv::Scalar(0, 0, 255), 2, 1);
    // //grid_map.show("JPS Path (Red)", 0);
    
    // grid_map.drawPath(opt_pixel_path, cv::Scalar(0, 255, 0), 2, 1);
    // grid_map.show("MINCO Optimized (Green)", 0);
    
    return 0;
}