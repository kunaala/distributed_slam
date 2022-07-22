#ifndef MAP_PUB_HPP_
#define MAP_PUB_HPP_
#include <iostream>
#include <vector>
#include <Eigen/Core>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"





class MapPub{
    public:
        MapPub(ros::NodeHandle &nh);
        void update_odom(Eigen::VectorXf pose);
        void update_map(std::vector<int8_t> rer);
        nav_msgs::Odometry odom;
        nav_msgs::OccupancyGrid map;
        sensor_msgs::LaserScan scan;
        


    private: 
        ros::NodeHandle nh_;
        ros::Publisher scan_pub, odom_pub, map_pub;
        unsigned int num_readings;

        tf2_ros::TransformBroadcaster br_;
        geometry_msgs::TransformStamped world_T_map,world_T_robot;
        // Helper function
        geometry_msgs::Quaternion rpy_to_q(double roll, double pitch, double yaw){
            tf2::Quaternion q_tf2;
            q_tf2.setRPY(roll, pitch, yaw);
            geometry_msgs::Quaternion q;
            tf2::convert(q_tf2, q);
            return q;
        }

};
#endif //MAP_PUB_HPP_