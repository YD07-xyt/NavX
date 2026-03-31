#include"../include/a_star.h"
#include <algorithm>
#include <cmath>
namespace planner {
    std::vector<Point2d> Astar::search(Point2d start,
        Point2d end,
        const std::vector<std::vector<int>>& global_map)
    {
        g_score[start] = 0.0;
        open_list.push({start, 0.0});
        this->neighbors = {// 邻居节点向量
            Point2d(1, 0),
            Point2d(-1, 0),
            Point2d(0, 1),
            Point2d(0, -1),
            Point2d(1, 1),
            Point2d(-1, 1),
            Point2d(-1, -1),
            Point2d(1, -1)};
        while (!open_list.empty()) {
            Node current = open_list.top();
            if (current.point == end) {
                trace_back(current.point, start);
                break;
            }
            if (close_list.contains(current.point)) {
                continue;
            }
            close_list.insert(current.point);

            for (auto& n: this->neighbors) {
                Point2d neighbor = {current.point.x() + n.x(),
                    current.point.y() + n.y()};
                int nx = neighbor.x();
                int ny = neighbor.y();
                if (nx < 0 || nx >= static_cast<int>(global_map[0].size()) ||
                    ny < 0 || ny >= static_cast<int>(global_map.size())) {
                    continue;
                }
                if (global_map[nx][ny] != 0) {
                    continue;
                }
                double new_g = g_score[current.point] +
                               sqrt(n.x() * n.x() + n.y() * n.y());
                if (!g_score.contains(neighbor) || new_g < g_score[neighbor]) {
                    g_score[neighbor] = new_g;
                    double f = new_g + heuristic(neighbor, end);
                    open_list.push({neighbor, f});
                    this->path[neighbor] = neighbor;
                }
            }
        }
        return {};
    };

    std::vector<Point2d> Astar::trace_back(Point2d current_point,
        Point2d start_point)
    {
        std::vector<Point2d> path_sum;
        while (path.find(current_point) != path.end()) {
            path_sum.push_back(current_point);
            current_point = path[current_point];
        }
        path_sum.push_back(start_point);
        std::reverse(path_sum.begin(), path_sum.end());
        return path_sum;
    };

    double heuristic(Point2d node1, Point2d node2)
    {
        int d1 = node1.x() - node2.x();
        int d2 = node1.y() - node2.y();
        return std::sqrt(static_cast<double>(d1 * d1 + d2 * d2));
    };
}// namespace global_planner