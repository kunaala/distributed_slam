#ifndef GAUSSIAN_PUB_HPP_
#define GAUSSIAN_PUB_HPP_
#include<iostream>
#include<vector>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>




class Gaussian_pub{
    public:
        Gaussian_pub(const ros::NodeHandle &nh);
        void update_odom(std::vector<double> pose);
        void update_map(std::vector<int8_t> rer);
        nav_msgs::Odometry odom;
        nav_msgs::OccupancyGrid map;
        sensor_msgs::LaserScan scan;
        // void odom_callback(const ros::TimerEvent &event);
        // void map_callback(const ros::TimerEvent &event);


    private: 
        ros::NodeHandle nh_;
        ros::Publisher scan_pub, odom_pub, map_pub;

        unsigned int num_readings;

        tf::TransformBroadcaster br;
        tf::Transform world_T_map,world_T_robot;
        ros::Timer timer_odom_,timer_map_;




};
#endif //GAUSSIAN_PUB_HPP_