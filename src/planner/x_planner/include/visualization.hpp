#pragma once
#include <opencv2/opencv.hpp>
#include <opencv4/opencv2/core/types.hpp>
#include <string>
#include <vector>
#include <queue>
#include <cmath>
#include "base.h"

namespace test {

// 计算距离场的数据结构
struct GridNode {
    int x, y;
    double dist;
    GridNode(int x_, int y_, double d) : x(x_), y(y_), dist(d) {}
    bool operator>(const GridNode& other) const { return dist > other.dist; }
};

class SimpleOccupancyMap {
private:
    cv::Mat map_;                    // 原始二值地图
    cv::Mat display_map_;           // 显示用彩色地图
    cv::Mat distance_map_;          // 距离场
    cv::Mat gradient_x_;            // X方向梯度
    cv::Mat gradient_y_;            // Y方向梯度
    int width_, height_;
    bool esdf_computed_ = false;
    
public:
    SimpleOccupancyMap(const std::string& png_path) {
        read(png_path);
        computeESDF();  // 自动计算ESDF
    }
    
    bool read(const std::string& png_path) {
        map_ = cv::imread(png_path, cv::IMREAD_GRAYSCALE);
        if (map_.empty()) return false;
        width_ = map_.cols;
        height_ = map_.rows;
        
        // 创建彩色显示地图
        cv::cvtColor(map_, display_map_, cv::COLOR_GRAY2BGR);
        
        std::cout << "地图加载成功: " << width_ << " x " << height_ << std::endl;
        return true;
    }
    
    // ========== ESDF计算 ==========
    void computeESDF() {
        std::cout << "计算ESDF距离场..." << std::endl;
        
        // 初始化距离场
        distance_map_ = cv::Mat(height_, width_, CV_64FC1, cv::Scalar(1e9));
        gradient_x_ = cv::Mat(height_, width_, CV_64FC1, cv::Scalar(0));
        gradient_y_ = cv::Mat(height_, width_, CV_64FC1, cv::Scalar(0));
        
        // 使用多源BFS计算距离场
        std::priority_queue<GridNode, std::vector<GridNode>, std::greater<GridNode>> pq;
        
        // 初始化：障碍物点距离为0
        for (int y = 0; y < height_; y++) {
            for (int x = 0; x < width_; x++) {
                if (isObstacle(planner::Point(x, y))) {
                    distance_map_.at<double>(y, x) = 0.0;
                    pq.push(GridNode(x, y, 0.0));
                }
            }
        }
        
        // BFS方向（8方向）
        const int dx[] = {-1, -1, -1, 0, 0, 1, 1, 1};
        const int dy[] = {-1, 0, 1, -1, 1, -1, 0, 1};
        
        // Dijkstra算法计算精确欧几里得距离
        while (!pq.empty()) {
            GridNode current = pq.top();
            pq.pop();
            
            double current_dist = current.dist;
            if (current_dist > distance_map_.at<double>(current.y, current.x) + 1e-6) {
                continue;
            }
            
            for (int dir = 0; dir < 8; dir++) {
                int nx = current.x + dx[dir];
                int ny = current.y + dy[dir];
                
                if (nx >= 0 && nx < width_ && ny >= 0 && ny < height_) {
                    // 欧几里得距离增量
                    double step_dist = std::sqrt(dx[dir]*dx[dir] + dy[dir]*dy[dir]);
                    double new_dist = current_dist + step_dist;
                    
                    if (new_dist < distance_map_.at<double>(ny, nx) - 1e-6) {
                        distance_map_.at<double>(ny, nx) = new_dist;
                        pq.push(GridNode(nx, ny, new_dist));
                    }
                }
            }
        }
        
        // 计算梯度（使用有限差分）
        for (int y = 1; y < height_ - 1; y++) {
            for (int x = 1; x < width_ - 1; x++) {
                double dx = (distance_map_.at<double>(y, x+1) - distance_map_.at<double>(y, x-1)) / 2.0;
                double dy = (distance_map_.at<double>(y+1, x) - distance_map_.at<double>(y-1, x)) / 2.0;
                
                gradient_x_.at<double>(y, x) = dx;
                gradient_y_.at<double>(y, x) = dy;
            }
        }
        
        // 边界处理
        // 边界处理
        for (int y = 0; y < height_; y++) {
            gradient_x_.at<double>(y, 0) = gradient_x_.at<double>(y, 1);
            gradient_x_.at<double>(y, width_-1) = gradient_x_.at<double>(y, width_-2);
        }

        for (int x = 0; x < width_; x++) {
            gradient_y_.at<double>(0, x) = gradient_y_.at<double>(1, x);
            gradient_y_.at<double>(height_-1, x) = gradient_y_.at<double>(height_-2, x);
        }
        
        esdf_computed_ = true;
        std::cout << "ESDF计算完成" << std::endl;
    }
    
    // ========== ESDF查询接口 ==========
    
    // 获取到最近障碍物的距离
    double getDistance(double x, double y) const {
        if (!esdf_computed_) return 1e9;
        
        int ix = std::round(x);
        int iy = std::round(y);
        
        if (ix < 0 || ix >= width_ || iy < 0 || iy >= height_) {
            // 边界外：返回到地图边界的距离
            double dx = std::min(x, 0.0) + std::max(x - (width_-1), 0.0);
            double dy = std::min(y, 0.0) + std::max(y - (height_-1), 0.0);
            return std::sqrt(dx*dx + dy*dy);
        }
        
        return distance_map_.at<double>(iy, ix);
    }
    
    // 获取距离梯度（指向远离障碍物的方向）
    cv::Point2d getGradient(double x, double y) const {
        if (!esdf_computed_) return cv::Point2d(0, 0);
        
        int ix = std::round(x);
        int iy = std::round(y);
        
        if (ix < 0 || ix >= width_ || iy < 0 || iy >= height_) {
            return cv::Point2d(0, 0);
        }
        
        double gx = gradient_x_.at<double>(iy, ix);
        double gy = gradient_y_.at<double>(iy, ix);
        
        // 归一化梯度（指向远离障碍物的方向）
        double norm = std::sqrt(gx*gx + gy*gy);
        if (norm > 1e-6) {
            gx /= norm;
            gy /= norm;
        }
        
        return cv::Point2d(gx, gy);
    }
    
    // 同时获取距离和梯度
    void getDistanceAndGradient(double x, double y, double& dist, cv::Point2d& grad) const {
        dist = getDistance(x, y);
        grad = getGradient(x, y);
    }
    
    // 检查是否碰撞（带安全距离）
    bool isCollision(double x, double y, double safety_margin = 0.3) const {
        double dist = getDistance(x, y);
        return dist < safety_margin;
    }
    
    // 获取有符号距离（障碍物内为负）
    double getSignedDistance(double x, double y) const {
        double dist = getDistance(x, y);
        if (isObstacle(planner::Point(std::round(x), std::round(y)))) {
            return -dist;
        }
        return dist;
    }
    
    // ========== 原始地图查询 ==========
    
    bool isObstacle(const planner::Point& point) const {
        int x = point.x;
        int y = point.y;
        
        if (x < 0 || x >= width_ || y < 0 || y >= height_) {
            return true;
        }
        
        return map_.at<uchar>(y, x) < 128;
    }
    
    bool isInMap(const planner::Point& point) const {
        return (point.x >= 0 && point.x < width_ && 
                point.y >= 0 && point.y < height_);
    }
    
    // ========== 可视化 ==========
    
    void visualizeESDF(const std::string& window_name = "ESDF Distance Map") {
        if (!esdf_computed_) return;
        
        cv::Mat esdf_viz(height_, width_, CV_8UC3);
        double max_dist = 20.0;  // 最大显示距离
        
        for (int y = 0; y < height_; y++) {
            for (int x = 0; x < width_; x++) {
                double dist = distance_map_.at<double>(y, x);
                dist = std::min(dist, max_dist);
                int intensity = (int)(255 * (1.0 - dist / max_dist));
                
                // 热力图：红色=近（障碍物），蓝色=远（自由空间）
                esdf_viz.at<cv::Vec3b>(y, x) = cv::Vec3b(
                    intensity,           // B
                    intensity / 2,       // G
                    255 - intensity      // R
                );
            }
        }
        
        cv::imshow(window_name, esdf_viz);
        cv::waitKey(0);
    }
    
    void drawPath(const std::vector<planner::Point>& path, 
                const cv::Scalar& color = cv::Scalar(0, 0, 255),
                int point_radius = 0,
                int line_thickness = 1) {
        if (path.empty()) return;
        
        // 绘制连线
        for (size_t i = 0; i < path.size() - 1; ++i) {
            cv::Point pt1(path[i].x, path[i].y);
            cv::Point pt2(path[i+1].x, path[i+1].y);
            
            if (isInMap(path[i]) && isInMap(path[i+1])) {
                cv::line(display_map_, pt1, pt2, color, line_thickness, cv::LINE_AA);
            }
        }
        
        // 绘制起点和终点
        if (!path.empty()) {
            cv::Point start_pt(path[0].x, path[0].y);
            cv::Point end_pt(path[path.size()-1].x, path[path.size()-1].y);
            
            cv::rectangle(display_map_, cv::Rect(start_pt.x - 1, start_pt.y - 1, 3, 3), 
                         cv::Scalar(0, 255, 0), -1);
            cv::rectangle(display_map_, cv::Rect(end_pt.x - 1, end_pt.y - 1, 3, 3), 
                         cv::Scalar(255, 0, 0), -1);
        }
    }
    
    void show(const std::string& window_name = "Pixel Grid Navigation", int wait_time = 0) {
        cv::namedWindow(window_name, cv::WINDOW_NORMAL);
        cv::resizeWindow(window_name, 800, 600);
        cv::imshow(window_name, display_map_);
        cv::waitKey(wait_time);
    }
    
    void save(const std::string& filename) {
        cv::imwrite(filename, display_map_);
    }
    
    void resetDisplay() {
        cv::cvtColor(map_, display_map_, cv::COLOR_GRAY2BGR);
    }
    
    // 获取原始地图
    cv::Mat getRawMap() const { return map_; }
    cv::Mat getDistanceMap() const { return distance_map_; }
    int getWidth() const { return width_; }
    int getHeight() const { return height_; }
};

} // namespace test