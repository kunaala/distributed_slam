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
#include "multiagent_slam/map2D.h"


using namespace Eigen;
std::fstream filter_data(std::string infile, std::string outfile){
    std::ifstream fd1(infile,std::ios::in);
    std::fstream fd2(outfile,std::ios::out);

    if(fd1.is_open()) {
        std::string test;
        while (!fd1.eof()){
            std::getline(fd1,test,' ');
            if (test.find("FLASER") != std::string::npos){
                std::getline(fd1,test);
                // for (auto i : msg) std::cout<<i<<'\n';
                fd2<<test;
                fd2<<'\n';
            }
            else if (test.find("ODOM") != std::string::npos){
                std::getline(fd1,test);
                // for (auto i : msg) std::cout<<i<<'\n';
                fd2<<test;
                fd2<<'\n';
            }
        }

        
        fd1.close();
        fd2.close();
    }

    else std::cerr<<"file not found"<<'\n';
    return fd2;
}
void save_data(MatrixXd m, std::string filename ){

    /*
     * Saves Eigen map "m" in the file specified in "filename" in CSV format.
     */
    const static IOFormat CSVFormat(FullPrecision, DontAlignCols, ", ");

	std::ofstream fd(filename,std::ios::out);
	if (fd.is_open()){
		fd << m.format(CSVFormat);
        fd <<'\n';
		fd.close();
	}
}

std::vector<std::string> split_line(std::string str){
  std::vector<std::string> v;
  std::string word=""; 
  for(auto i : str){
    if(i == ' ') {
      v.push_back(word);
      word="";
    }
    else word = word+i;
  }
  return v;
}
std::vector<double> slice_data(std::string str){
  std::vector<double> v;
  std::string word=""; 
  

  for(auto i : str){
    if(i == ' ') {
      try{
        v.push_back(std::stod(word));
        word="";
      }
      catch(...){}
    }
    else word = word+i;
  }
  return v;
}

MatrixXd polar_cart(MatrixXd r, MatrixXd theta){
    // MatrixXd z(4,r.cols());
    MatrixXd z(2,r.cols());

    z.row(0) = (r.array() * theta.array().cos()).matrix();
    z.row(1) = (r.array() * theta.array().sin()).matrix();
    // z.row(2) = VectorXd::Zero(r.cols());
    // z.row(3) = VectorXd::Ones(r.cols());
    return z;
}

double tsdf(double p1_x, double p1_y, double p2_x, double p2_y, double qx, double qy){
    double d = abs((p2_x - p1_x) * (p1_y- qy) - (p1_x - qx)*(p2_y - p1_y));
    d /= sqrt(pow((p2_x - p1_x),2) + pow((p2_y - p1_y),2));
    return d;
}
double scale(double z, double resol){
  z/=resol;
  return z;

}

std::vector<std::pair<double,double> > getPseudoPts(double x, double y, double resolution) {
    std::vector<std::pair<double,double> > res;
    res.push_back(std::make_pair(x,y));
    res.push_back(std::make_pair(x+resolution,y));
    res.push_back(std::make_pair(x,y+resolution));
    res.push_back(std::make_pair(x+resolution,y+resolution));
    res.push_back(std::make_pair(x-resolution,y));
    res.push_back(std::make_pair(x,y-resolution));
    res.push_back(std::make_pair(x-resolution,y-resolution));
    res.push_back(std::make_pair(x+resolution,y-resolution));
    res.push_back(std::make_pair(x-resolution,y+resolution));
    return res;
}

double getSignedDist(double x0, double y0,double x1, double y1,double x2, double y2) {
    double dist = sqrt((x2-x1)*(x2-x1) + (y1-y2)*(y1-y2));
    // std::cout<<(dist)<<"\n";
    double val = abs((x2-x1)*(y1-y0) - (x1-x0)*(y2-y1));
    // std::cout<<val<<"\n";
    return val/dist;
}

double euclidDist(double x1, double y1, double x2, double y2) {
    return pow(x1-x2,2)+pow(y1-y2,2);
}


int main(int argc, char** argv){
  ros::init(argc, argv, "laser_scan_publisher");
  ros::NodeHandle n;
  ros::Publisher scan_pub = n.advertise<sensor_msgs::LaserScan>("scan", 50);
  ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom",50);
  ros::Publisher map_pub = n.advertise<nav_msgs::OccupancyGrid>("map",50);
  ros::Rate r(50.0);
  // filter_data("intel.gfs.log","filtered2.log");
  
  std::ifstream f("filtered2.log",std::ios::in);
  /* COMPOSING HEADER */
  // std::string header;   
  // std::getline(fd,header);
  // std::vector<std::string>  msg_h = split_line(header);
  std::unordered_map<std::string, double> param_map;
  //   unsigned int params = 8;
  //   for(unsigned int i =0;i<params;i++){
  //       param_map[msg_h.at(i)] = msg_val.at(i);
  //   }
  std::string data;
  
  param_map["start_angle"] = -1.570796;
  param_map["field_of_view"] = 3.141593;
  param_map["end_angle"] = param_map["start_angle"] + param_map["field_of_view"];
  param_map["angular_resolution"] = 0.017453294;
  param_map["maximum_range"] = 81.920000;
  param_map["laser_frequency"] = 40;

  std::vector<double> curr_odom={0,0};
  nav_msgs::Odometry odom;
  odom.header.frame_id = "world";
  odom.child_frame_id = "laser_frame";
  double x,y,theta,vx,vy,vtheta;



  std::vector<double> msg_val;
  sensor_msgs::LaserScan scan;
  scan.header.frame_id = "laser_frame";
  scan.angle_min = param_map["start_angle"] + theta; // pose correction
  scan.angle_max = (scan.angle_min + param_map["field_of_view"]) ; 
  scan.angle_increment = param_map["angular_resolution"] ; 
  unsigned int num_readings = (scan.angle_max - scan.angle_min)/scan.angle_increment;
  scan.time_increment = (1 / param_map["laser_frequency"]) / (num_readings);
  scan.range_min = 0.0;
  scan.range_max = param_map["maximum_range"];
  
  

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
  MatrixXd angles = VectorXd::LinSpaced(num_readings, param_map["start_angle"],param_map["end_angle"] ).transpose() ;


  world_T_map.setOrigin( tf::Vector3(0.0, 0.0, 0.0) );
  tf::Quaternion w_q_m;
  w_q_m.setRPY( 0, 0, 0 );  
  world_T_map.setRotation( w_q_m);
  br.sendTransform(tf::StampedTransform(world_T_map, ros::Time::now(), "world","map" ));

  map2D map_2d(0.0,0.0, resol, 300,300);
  int idx=0;
  while(n.ok() || !f.eof()){
    std::vector<double> ranges;
    std::getline(f,data);
    idx++;
    if(idx%200==0) std::cout<<idx<<"\n";
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
                
                MatrixXd zw = (T * ptBody/resol) + p/resol;
                map_2d.setValue(zw(0),zw(1),gVal);

            }
        }

        std::vector<int8_t> map_data = map_2d.getGrid();

        map.data = map_data;
        map_pub.publish(map);
        world_T_map.setOrigin( tf::Vector3(0.0, 0.0, 0.0) );
        tf::Quaternion w_q_m;
        w_q_m.setRPY( 0, 0, -3.14);  
        world_T_map.setRotation( w_q_m);
        br.sendTransform(tf::StampedTransform(world_T_map, ros::Time::now(), "world","map" ));



  // while(n.ok() && !f.eof()){
  //   std::vector<double> ranges;
  //   std::getline(f,data);
  //   msg_val = slice_data(data);

  //   if (msg_val.at(0) == 180){
  //       /* populate the LaserScan message */
  //       for (unsigned int i=1;i<msg_val.size()-7;++i)    ranges.push_back(msg_val.at(i));
  //       // for (auto i : ranges) std::cout<<i<<'\n';
  //       // std::cout<<ranges.size()<<'\n';
  //       ros::Time scan_time = ros::Time::now();
  //       scan.header.stamp = scan_time;

  //       scan.ranges.resize(num_readings);
  //       // std::cout<<"im here"<<'\n';
  //       for(unsigned int i = 0; i < num_readings; ++i){
        //   if(ranges[i] < 80 && ranges[i] >0.2){
        //     scan.ranges[i] = ranges[i] ;//+ curr_odom.at(0);// pose correction
        //   }
        // }
        // // std::cout<<scan.ranges.size()<<'\n';
        // scan_pub.publish(scan);
        // Map<MatrixXd> scans(ranges.data(),1,ranges.size());
        // for(int r=0;r<ranges.size();r++){
        //     if (scans(0,r) >80) scans(0,r) =0; 
        // }
        // MatrixXd z = polar_cart(scans, angles);
        // Vector2d p;
        // p << x,
        //      y;
        // MatrixXd zw = (T * z).colwise() + p;
        //     // MatrixXd zw = world_T_robot * z;
        // std::cout<<"updated map"<<'\n';                
 
        // // save_data(M,"map_ros.txt");
        // // std::vector<int8_t> map_data(M.data(), M.data() + M.rows() * M.cols());
        // std::vector<int8_t> map_data = getGrid();
        // map.data = map_data;
        // map_pub.publish(map);
        // world_T_map.setOrigin( tf::Vector3(0.0, 0.0, 0.0) );
        // tf::Quaternion w_q_m;
        // w_q_m.setRPY( 0, 0, 0 );  
        // world_T_map.setRotation( w_q_m);
        // br.sendTransform(tf::StampedTransform(world_T_map, ros::Time::now(), "world","map" ));
        


        
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
 
  //   // std::unordered_map<std::string, double>::const_iterator itr;
  //   // for(itr = param_map.begin();itr!=param_map.end();itr++) std::cout<<itr->first<<":"<<itr->second<<'\n';
  //   std::vector<double> ranges;

  //   for (unsigned int i=8;i<msg_val.size()-2;i++)    ranges.push_back(msg_val.at(i));
    r.sleep();
  }
}
