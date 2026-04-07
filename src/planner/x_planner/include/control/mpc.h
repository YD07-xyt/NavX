#pragma once
#include<Eigen/Core>
#include <Eigen/src/Core/Matrix.h>
namespace control{

struct Omni {
    Eigen::Vector<double,5> state;//px,py,theta,vx,vy
    Eigen::Vector3d control;//vx,vy,wz
};

class Mpc{
    public:
};


}