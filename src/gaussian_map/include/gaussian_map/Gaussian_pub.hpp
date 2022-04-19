#include<iostream>
#include<fstream>
#include<sstream>
#include<vector>
#include<Eigen/Dense>
#include<map>
#include<unordered_map>
#include <random>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include "gaussian_map/map2D.h"
#include "gaussian_map/gpModel.h"
#include "gaussian_map/utils/plot_utils.hpp"
#include "gaussian_map/utils/filter_utils.hpp"
#include "gaussian_map/utils/math_utils.h"

using namespace Eigen;


class Gaussian_pub{
    public:
        Gaussian_pub(ros::Nodehandle &nh);

    private: 
        ros::NodeHandle nh;
        ros::Publisher scan_pub, odom_pub, map_pub;
        ros::Rate r(50.0);  
        double end_angle, start_angle, field_of_view, angular_resolution, maximum_range, laser_frequency;
        std::string  data_file;
        std::ifstream file_name;
        nav_msgs::Odometry odom;
        nav_msgs::OccupancyGrid map;
        tf::TransformBroadcaster br;
        tf::Transform world_T_map,world_T_robot;
        Matrix2d T;
        MatrixXd angles
        int local_cx = 1,local_cy =1;
        double x_max=0.0, y_max = 0.0;
        double xL = 300.0, yL = 300.0;



 


};