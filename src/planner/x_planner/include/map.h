#pragma once
#include<Eigen/Core>
#include <eigen3/Eigen/src/Core/Matrix.h>
#include <opencv4/opencv2/core/types.hpp>
#include"visualization.hpp"
namespace planner {
    class Map {
        public: 
            Map(test::SimpleOccupancyMap vis_map):vis_map_(vis_map){
            }
        private:
            test::SimpleOccupancyMap vis_map_;
        public: 
            double getDistance(double x, double y){
                return vis_map_.getDistance(x, y);
            };
            Eigen::Vector2d getGradient(double x, double y){
                cv::Point2d p= vis_map_.getGradient(x, y);
                return Eigen::Vector2d(p.x,p.y);
            };
            bool isObstacle(planner::Point point){
                return vis_map_.isObstacle(point);
            };
            bool is_arrive(Point current,Point goal){
                return true;
            };
            bool is_in_map(Point current){
                return vis_map_.isInMap(current);
            };
             double getDistWithGradBilinear( Eigen::Vector2d bpt,Eigen::Vector2d gradESDF2d, double safeDis);
              double  getDistWithGradBilinear(Eigen::Vector2d pos);
                double getDistanceReal(Eigen::Vector2d pos) ;
    };
}