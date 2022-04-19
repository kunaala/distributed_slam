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



int main(int argc, char** argv){
  ros::init(argc, argv, "laser_scan_publisher");
  ros::NodeHandle nh;

  ros::Publisher scan_pub = nh.advertise<sensor_msgs::LaserScan>("scan", 50);
  ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("odom",50);
  ros::Publisher map_pub = nh.advertise<nav_msgs::OccupancyGrid>("map",50);
  ros::Rate r(50.0);
  // filter_data("intel.gfs.log","filtered2.log");
  std::ifstream f("filtered2.log",std::ios::in);

  std::string data;
  double end_angle, start_angle, field_of_view, angular_resolution, maximum_range, laser_frequency;
  nh.getParam("start_angle", start_angle);
  nh.getParam("field_of_view", field_of_view);
  nh.getParam("end_angle", end_angle);
  nh.getParam("angular_resolution", angular_resolution);
  nh.getParam("maximum_range", maximum_range);
  nh.getParam("laser_frequency", laser_frequency);
 
  std::vector<double> curr_odom={0,0};
  nav_msgs::Odometry odom;
  odom.header.frame_id = "world";
  odom.child_frame_id = "laser_frame";
  double x,y,theta,vx,vy,vtheta;



  std::vector<double> msg_val;
  sensor_msgs::LaserScan scan;
  scan.header.frame_id = "laser_frame";
  scan.angle_min = start_angle + theta; // pose correction
  scan.angle_max = scan.angle_min + field_of_view ; 
  scan.angle_increment = angular_resolution ; 
  unsigned int num_readings = (scan.angle_max - scan.angle_min)/scan.angle_increment;
  scan.time_increment = (1 / laser_frequency) / (num_readings);
  scan.range_min = 0.0;
  scan.range_max = maximum_range;
  
  

  nav_msgs::OccupancyGrid map;
  map.info.width =6000;
  map.info.height = 6000;
  map.info.resolution = 0.1;
  double resol = 0.1;
  map.header.frame_id = "map";
  map.info.origin.position.x = -300;
  map.info.origin.position.y = -300;
  map.info.origin.position.z = 0.0;
  double x_max=0.0, y_max = 0.0;


  
  map.info.origin.orientation = tf::createQuaternionMsgFromYaw(0);
  // MatrixXd M((int)(map.info.height),(int)(map.info.width));


  tf::TransformBroadcaster br;
  tf::Transform world_T_map,world_T_robot;
  int local_cx = 1,local_cy =1;
  Matrix2d T;
  MatrixXd angles = VectorXd::LinSpaced(num_readings, start_angle, end_angle ).transpose() ;


  world_T_map.setOrigin( tf::Vector3(0.0, 0.0, 0.0) );
  // tf::Quaternion w_q_m;
  // w_q_m.setRPY( 0, 0, 0 );  
  // world_T_map.setRotation( w_q_m);
  // br.sendTransform(tf::StampedTransform(world_T_map, ros::Time::now(), "world","map" ));
  double xL = 300.0, yL = 300.0;
  map2D map_2d(0.0,0.0, resol, xL,yL);
  int idx=0;
  while(n.ok() || !f.eof()){
    std::vector<double> ranges;
    std::getline(f,data);
    msg_val = slice_data(data);
    if (msg_val.at(0) == 180){
    
        Vector2d p(x,y); // robot pose
        /* populate the LaserScan message */
        for (unsigned int i=1;i<msg_val.size()-7;++i)    ranges.push_back(msg_val.at(i));
        scan.ranges.resize(num_readings);
        // for (auto i : ranges) std::cout<<i<<'\n';

        // std::cout<<ranges.size()<<'\n';
        ros::Time scan_time = ros::Time::now();
        scan.header.stamp = scan_time;

        
        // std::cout<<"im here"<<'\n';
        for(unsigned int i = 0; i < num_readings; ++i){
          if(ranges[i] > 80 || ranges.at(i) <0.2){
            ranges.at(i) = 0; 
            scan.ranges[i] = ranges[i] ;// pose correction
            // std::cout<<ranges.at(i)<<'\n';
          }
        }
        Map<MatrixXd> scans(ranges.data(),1,ranges.size());

        
        scan_pub.publish(scan);
        MatrixXd z = polar_cart(scans, angles);
        
        for(int k=0;k<ranges.size()-1;k++){
            double pt_x1 = z(0,k),  pt_y1 = z(1,k);
            double pt_x2 = z(0,k+1), pt_y2 = z(1,k+1);
            std::vector<std::pair<double,double> > psPts = getPseudoPts(pt_x1,pt_y1,resol);

            for(int j=0;j<9;j++) {
                double gVal = getSignedDist(psPts[j].first,psPts[j].second,pt_x1,pt_y1,pt_x2,pt_y2);
                if(euclidDist(psPts[j].first,psPts[j].second,0,0)>euclidDist(pt_x1,pt_y1,0,0)) {
                    gVal = -gVal;
                }
                Vector2d ptBody(psPts[j].first,psPts[j].second);
                
                // MatrixXd zw = (T * ptBody/resol) + p/resol;
                MatrixXd zw = (T * ptBody) + p;

                // std::cout<<zw(0)<<'\t'<<zw(1)<<'\n';
                map_2d.setValue(zw(0),zw(1),gVal);

            }
        }

        
        double sigma_noise, l, c, up, down;
        std::string line;
        int dim,data_size, pseudo_size, test_size;
        std::ifstream configFile ("params");
        if (configFile.is_open()) {
            getline(configFile,line);
            c = stod(line);
            getline(configFile,line);
            l = stod(line);
            getline(configFile,line);
            sigma_noise = stod(line);
        }
        configFile.close();
        // std::ofstream dataFile;
        // dataFile.open("data.csv");
        std::pair<Eigen::MatrixXd,Eigen::VectorXd> psData = map_2d.getPseudoPts();
        std::pair<Eigen::MatrixXd,Eigen::VectorXd> trData = map_2d.getTrainingPts();
      
        gpModel model(c,l,sigma_noise);
        model.set_training_data(trData.first,trData.second);
        model.set_pseudo_data(psData.first);
        
        std::random_device rd{};
        std::mt19937 gen{rd()};
        std::normal_distribution<> rand{0, 1.0};
        int temp = psData.second.size();
        Eigen::MatrixXd X_test(temp,2);
  
        for(int i=0;i<temp;i++) {
            // for(int j=0;j<temp;j++) {
            //     // std::cout<<i*temp + j<<"\n";
            X_test(i,0) = psData.first(i,0) + rand(gen);
            X_test(i,1) = psData.first(i,1) + rand(gen);
            // dataFile << X_test(i,0)<<" "<<X_test(i,1)<<" ";
            // }
        }
        model.set_test_data(X_test);
        std::tuple<Eigen::VectorXd, Eigen::VectorXd> res = model.predict();

        Eigen::VectorXd mu_pred = std::get<0>(res);
        Eigen::VectorXd sigma_pred = std::get<1>(res);

    
        std::vector<int8_t> rer = getDistGrid(X_test,mu_pred,resol,xL,yL);
        map.data = rer;
        map_pub.publish(map);
        world_T_map.setOrigin( tf::Vector3(0.0, 0.0, 0.0) );
        tf::Quaternion w_q_m;
        w_q_m.setRPY( 0, 0, 0);  
        world_T_map.setRotation( w_q_m);
        br.sendTransform(tf::StampedTransform(world_T_map, ros::Time::now(), "world","map" ));
        std::cout<<"once"<<'\n';
 
    }
  
    else{
      
      x = msg_val.at(0);
      y = msg_val.at(1);
      theta = msg_val.at(2);
      world_T_robot.setOrigin( tf::Vector3(x/resol, y/resol, 0.0) );
      tf::Quaternion w_q_r;
      w_q_r.setRPY( 0, 0, theta );  
      world_T_robot.setRotation( w_q_r);
      
      vx = msg_val.at(3);
      vy = msg_val.at(4);
      vtheta = msg_val.at(5);
      odom.header.stamp = ros::Time::now();
      odom.pose.pose.position.x = x/resol;
      odom.pose.pose.position.y = y/resol;
      odom.pose.pose.position.z = 0.0;
      odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(theta);
      //set the velocity
      odom.twist.twist.linear.x = vx;
      odom.twist.twist.linear.y = vy;
      odom.twist.twist.angular.z = vtheta;

      //publish the message
      odom_pub.publish(odom);
      T << cos(theta), -sin(theta),
            sin(theta), cos(theta);
      br.sendTransform(tf::StampedTransform(world_T_robot, ros::Time::now(), "world","laser_frame" ));


    }
 
    r.sleep();
  }
  
    return 0;
}


