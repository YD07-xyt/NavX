#pragma once
#include <memory>
#include <queue>
#include <unordered_set>
#include<vector>
#include"../map.h"
#include<optional>
namespace planner {
    struct Node{
        Point point;
        std::shared_ptr<Node> parent;
        double g;  // 实际代价
        double h;  // 启发式代价
        double f;  // f = g + h
        bool operator>(const Node& other) const {
            return f > other.f;
        }
        bool operator==(const Node& other) const {
            return point.x == other.point.x && point.y == other.point.y;
        }
         Node(const Point& p) : point(p), parent(nullptr), g(0), h(0), f(0) {}
        Node() : point(0,0), parent(nullptr), g(0), h(0), f(0) {}
    };
    struct Neighbour{
        std::vector<Point> neighbour_;
        Neighbour(){
            neighbour_.reserve(8);
            neighbour_.emplace_back(0, -1);
            neighbour_.emplace_back(0, 1);
            neighbour_.emplace_back(-1, 0);
            neighbour_.emplace_back(1, 0);
            neighbour_.emplace_back(1, 1);
            neighbour_.emplace_back(1, -1);
            neighbour_.emplace_back(-1, 1);
            neighbour_.emplace_back(-1, -1);
        };
    };
    struct PairHash {
        std::size_t operator()(const Point& p) const noexcept {
            std::size_t seed = 0;
            seed ^= std::hash<int>{}(p.x) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
            seed ^= std::hash<int>{}(p.y) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
            return seed;
        }
    };
    class jps {
    public:
        jps(Map map);
        std::optional<std::vector<Point>> search(const Point& start,const Point& end);
        double heuristic(Point start,Point goal);

    private:
        std::shared_ptr<Map> map_;
        std::shared_ptr<std::priority_queue<Node, std::vector<Node>, std::greater<Node>>> open_list;
        std::shared_ptr<std::unordered_set<Point,PairHash>> close_list;
        std::shared_ptr<std::vector<Point>> neighbors_;
        std::vector<Point> trace_back(Node end_node);
        
        std::optional<Node>  Jump(Node current,Point direction,Point goal);
        bool isDiagonal(Point direction);
        bool hasForcedNeighbour(Point current,Point parent);
        int sign(int x) {
            if (x > 0) return 1;
            if (x < 0) return -1;
            return 0;
        }
        bool isObstacle(double x, double y);
        bool isFree(double x, double y); 
    };
}