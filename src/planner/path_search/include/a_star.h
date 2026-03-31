#pragma once

#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include <Eigen/Core>

#ifndef A_STAR_HPP
#define A_STAR_HPP

namespace planner {
    using Point2d = Eigen::Vector2d;

    struct Node {
        Point2d point;
        double f;
        bool operator>(const Node& other) const
        {
            return f > other.f;
        }
    };

    struct PairHash {
        std::size_t operator()(const Point2d& p) const noexcept
        {
            return std::hash<long long>()(
                ((long long) p.x() << 32) ^ (long long) p.y());
        }
    };

    struct Point2dEqual {
    bool operator()(const Point2d& a, const Point2d& b) const {
        return a == b;
        // return (a - b).norm() < 1e-9;
    }
    };
    double heuristic(Point2d node1, Point2d node2);


    std::vector<Point2d> trace_back();
    class Astar
    {
    public:
        std::vector<Point2d> search(Point2d start,
            Point2d end,
            //TODO 优化map传入
            const std::vector<std::vector<int>>& global_map);

    private:
        std::vector<Point2d> neighbors;
        std::priority_queue<Node, std::vector<Node>, std::greater<>> open_list;
        std::unordered_set<Point2d,PairHash> close_list;
        std::unordered_map<Point2d, Point2d, PairHash> path;
        std::unordered_map<Point2d, double, PairHash> g_score;
        std::vector<Point2d> trace_back(Point2d current_point,
            Point2d start_point);
    };
}// namespace global_planner
#endif//a_star