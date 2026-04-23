#pragma once

#include <string>
namespace decision {
    struct Point{
        double x;
        double y;
        double yaw;
        Point(double x,double y,double yaw):x(x),y(y),yaw(yaw){};
    };
    struct GoalPoint{
        Point Patrol1=Point(1,1,1);
        Point Patrol2=Point(2,2,2);
        Point home=Point(0.229,0.807,0.0);
    };
    struct FSMConfig{
        std::string map_tf_name_="map";

    };
    struct GohomeState{
        int hp=200;
        int projectile_allowance=0;
    };
    enum class Nav2State {
        unkown,//一般未开始
        aborted,
        succeeded,
        running,
    };
    enum class NavTask{
        free,
        wait,
        running,
    };
    enum class RobotTask {
        free,
        Patrol,
        run2home,
        go2goal,
    };
    enum class RobotState {
        supply,
        defense,
        attack,
        move,
    };
}