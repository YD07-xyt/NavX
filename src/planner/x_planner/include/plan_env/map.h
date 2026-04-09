#pragma once
#include<Eigen/Core>
#include <eigen3/Eigen/src/Core/Matrix.h>
#include <memory>
#include "../base.h"
#include"sdf_map.h"
namespace planner {
    class Map {
        public: 
            Map(MapConfig map_config,std::function<void()> pub_esdf_fun,std::function<void()> pub_gridmap_fun){
                sdf_map_=std::make_shared<SDFmap>(map_config,pub_esdf_fun,pub_gridmap_fun);
            }
        public:
            std::shared_ptr<SDFmap> sdf_map_;
        public: 
            double getDistance(double x, double y){
                return sdf_map_->getDistance(x,y);
            };

            bool isObstacle(planner::Point point){
                return sdf_map_->isOccupied(point.x,point.y);
            };
            //TODO goal是否可达
            bool is_arrive(Point current,Point goal){
                return true;
            };
            bool is_in_map(Point current){
                Eigen::Vector2d v;
                v<<current.x,current.y;
                return sdf_map_->isInGloMap(v);
            };
            double getDistWithGradBilinear( Eigen::Vector2d bpt,Eigen::Vector2d gradESDF2d, double safeDis){
                return sdf_map_->getDistWithGradBilinear(bpt,gradESDF2d,safeDis);
            };
            double  getDistWithGradBilinear(Eigen::Vector2d pos){
                return sdf_map_->getDistWithGradBilinear(pos);
            };
            double getDistanceReal(Eigen::Vector2d pos){
                return sdf_map_->getDistanceReal(pos);
            };
    };
}