#include "../../include/path_search/jps.h"
#include <cmath>
#include <memory>
#include <optional>
#include <vector>


namespace planner {

jps::jps(Map map) {
  map_ = std::make_shared<Map>(map);
  open_list = std::make_shared<
      std::priority_queue<Node, std::vector<Node>, std::greater<Node>>>();
  close_list = std::make_shared<std::unordered_set<Point, PairHash>>();
  neighbors_ =
      std::make_shared<std::vector<Point>>(std::initializer_list<Point>{
          Point(1, 0), Point(-1, 0), Point(0, 1), Point(0, -1), Point(1, 1),
          Point(-1, 1), Point(-1, -1), Point(1, -1)});
  close_list->reserve(100000);
}

std::optional<std::vector<Point>> jps::search(const Point &start,
                                              const Point &end) {
  Node start_node(start);
  Node end_node(end);

    
    std::cout << "开始搜索..." << std::endl;
    
    if (!map_->is_in_map(start) || !map_->is_in_map(end)){
        std::cerr << "起点或终点超出地图范围" << std::endl;
        return std::nullopt;
    }
    if (map_->isObstacle(start) || map_->isObstacle(end)){
        std::cerr << "起点或终点是障碍物" << std::endl;
        return std::nullopt;
    }
    
  open_list->push(start_node);

  while (!open_list->empty()) {
    Node current = open_list->top();
    open_list->pop();

    if (current.point == end) {
      return trace_back(current);
    }

    if (close_list->count(current.point)){
      continue;
    }
    close_list->insert(current.point);

    for (const Point &dir : *neighbors_) {
      auto jump_node = Jump(current, dir, end);
      if (!jump_node.has_value()){
        continue;
      }
      Node jp = jump_node.value();
      if (close_list->count(jp.point)){
        continue;
      }
      
      // 计算代价
      jp.g = current.g + 1.0f;
      jp.h = std::hypot(jp.point.x - end.x, jp.point.y - end.y);
      jp.f = jp.g + jp.h;

      open_list->push(jp);
    }
  }

  return std::nullopt;
}

std::optional<Node> jps::Jump(Node current, Point direction, Point goal) {
  // 下一步点
  Point next_pt = current.point + direction;

  if (!map_->is_in_map(next_pt) || map_->isObstacle(next_pt)) {
    return std::nullopt;
  }

  Node next_node;
  next_node.point = next_pt;
  next_node.parent = std::make_shared<Node>(current); 

  // 到达终点
  if (next_node.point == goal) {
    return next_node;
  }

  if (current.parent && hasForcedNeighbour(next_node.point, current.point)) {
    return next_node;
  }

  // 斜向：检查横竖正交方向
  if (isDiagonal(direction)) {
    Point h_dir(direction.x, 0);
    Point v_dir(0, direction.y);

    bool h = Jump(next_node, h_dir, goal).has_value();
    bool v = Jump(next_node, v_dir, goal).has_value();

    if (h || v) {
      return next_node;
    }
  }

  return Jump(next_node, direction, goal);
}

bool jps::isDiagonal(Point direction) {
  return direction.x != 0 && direction.y != 0;
}

bool jps::hasForcedNeighbour(Point current, Point from) {
  int dx = current.x - from.x;
  int dy = current.y - from.y;

  // 上
  if (dx == 0 && dy == -1) {
    return (isObstacle(current.x - 1, current.y) &&
            isFree(current.x - 1, current.y - 1)) ||
           (isObstacle(current.x + 1, current.y) &&
            isFree(current.x + 1, current.y - 1));
  }
  // 下
  if (dx == 0 && dy == 1) {
    return (isObstacle(current.x - 1, current.y) &&
            isFree(current.x - 1, current.y + 1)) ||
           (isObstacle(current.x + 1, current.y) &&
            isFree(current.x + 1, current.y + 1));
  }
  // 左
  if (dx == -1 && dy == 0) {
    return (isObstacle(current.x, current.y - 1) &&
            isFree(current.x - 1, current.y - 1)) ||
           (isObstacle(current.x, current.y + 1) &&
            isFree(current.x - 1, current.y + 1));
  }
  // 右
  if (dx == 1 && dy == 0) {
    return (isObstacle(current.x, current.y - 1) &&
            isFree(current.x + 1, current.y - 1)) ||
           (isObstacle(current.x, current.y + 1) &&
            isFree(current.x + 1, current.y + 1));
  }

  return false;
}

bool jps::isObstacle(double x, double y) {
  return !map_->is_in_map({x, y}) || map_->isObstacle({x, y});
}

bool jps::isFree(double x, double y) {
  return map_->is_in_map({x, y}) && !map_->isObstacle({x, y});
}


std::vector<Point> jps::trace_back(Node end_node) {
  std::vector<Point> path;
  Node cur = end_node;
  while (cur.parent != nullptr) {
    path.push_back(cur.point);
    cur = *cur.parent;
  }
  path.push_back(cur.point);
  std::reverse(path.begin(), path.end());
  return path;
}

} // namespace planner
