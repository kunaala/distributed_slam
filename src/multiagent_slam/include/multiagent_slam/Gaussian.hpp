#ifndef GAUSSIAN_H
#define GAUSSIAN_H

#include<iostream>
#include<fstream>
#include<sstream>
#include<vector>
#include<Eigen/Dense>
#include<map>
#include<unordered_map>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>

using namespace Eigen;

class Gaussian{
    public:
        Gaussian(const ros::NodeHandle &n);
        std::fstream filter_data(std::string infile, std::string outfile)
        void capture_data();
        void save_data(MatrixXd m, std::string filename );
        void timer_callback(const ros::TimerEvent &event);
        
        


    private:
        double tsdf(double p1_x, double p1_y, double p2_x, double p2_y, double qx, double qy);
        double scale(double z, double resol);
        std::vector<std::string> split_line(std::string str);
        std::vector<double> slice_data(std::string str);
        MatrixXd polar_cart(MatrixXd r, MatrixXd theta);
        ros::NodeHandle &n_;
        ros::Publisher scan_pub_;
        ros::Publisher odom_pub_;
        ros::Publisher map_pub_;
        
        ros::Timer timer_;

        std::unordered_map<std::string, double> param_map;
        sensor_msgs::LaserScan scan;
        nav_msgs::OccupancyGrid map;
        std::ifstream f("filtered2.log",std::ios::in);






};
#endif // GAUSSIAN_H




