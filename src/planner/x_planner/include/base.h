#pragma once
namespace planner {

    struct Point{
        double x;
        double y;
        Point operator+(const Point& other) const {
            return Point{x+other.x,y+other.y};
        }
        bool operator==(const Point& other) const {
            return x==other.x&&y==other.y;
        }
        Point(double x,double y):x(x),y(y){};
    };

}