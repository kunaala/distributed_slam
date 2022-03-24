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
  scan.angle_min = param_map["start_angle"] + curr_odom.at(1); // pose correction
  scan.angle_max = (scan.angle_min + param_map["field_of_view"]) ; 
  scan.angle_increment = param_map["angular_resolution"] ; 
  unsigned int num_readings = (scan.angle_max - scan.angle_min)/scan.angle_increment;
  scan.time_increment = (1 / param_map["laser_frequency"]) / (num_readings);
  scan.range_min = 0.0;
  scan.range_max = param_map["maximum_range"];
  
  

  nav_msgs::OccupancyGrid map;
  map.info.width =500;
  map.info.height = 500;
  map.info.resolution = 0.1;
  map.header.frame_id = "map";
  map.info.origin.position.x = -25;
  map.info.origin.position.y = -25;
  map.info.origin.position.z = 0.0;
  unsigned int cx = 25;
  unsigned int cy = 25;

  
  map.info.origin.orientation = tf::createQuaternionMsgFromYaw(0);
  MatrixXd M((int)(map.info.height),(int)(map.info.width));


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

  while(n.ok() && !f.eof()){
    std::vector<double> ranges;
    std::getline(f,data);
    msg_val = slice_data(data);

    if (msg_val.at(0) == 180){
        /* populate the LaserScan message */
        for (unsigned int i=1;i<msg_val.size()-7;++i)    ranges.push_back(msg_val.at(i));
        // for (auto i : ranges) std::cout<<i<<'\n';
        // std::cout<<ranges.size()<<'\n';
        ros::Time scan_time = ros::Time::now();
        scan.header.stamp = scan_time;

        scan.ranges.resize(num_readings);
        // std::cout<<"im here"<<'\n';
        for(unsigned int i = 0; i < num_readings; ++i){
          if(ranges[i] < 80 && ranges[i] >0.2){
            scan.ranges[i] = ranges[i] ;//+ curr_odom.at(0);// pose correction
          }
        }
        // std::cout<<scan.ranges.size()<<'\n';
        scan_pub.publish(scan);
        Map<MatrixXd> scans(ranges.data(),1,ranges.size());
        for(int r=0;r<ranges.size();r++){
            if (scans(0,r) >80) scans(0,r) =0; 
        }
        MatrixXd z = polar_cart(scans, angles);
        Vector2d p;
        p << x,
             y;
        MatrixXd zw = (T * z).colwise() + p;
            // MatrixXd zw = world_T_robot * z;


        for(int k=0;k<ranges.size()-1;k++){
            double p1_x = zw.col(k)(0),  p1_y = zw.col(k)(1);
            double p2_x = zw.col(k+1)(0),  p2_y = zw.col(k+1)(1);
            local_cx = int(zw.col(k)(0));
            local_cy = int(zw.col(k)(1));
            for(double i=local_cx-map.info.resolution;i<=local_cx+map.info.resolution;i+=map.info.resolution){
                for(double j=local_cy-map.info.resolution;j<=local_cy+map.info.resolution;j+=map.info.resolution){
                    double d = tsdf(p1_x,p1_y,p2_x,p2_y,i,j);
                    int map_x = (int)((cx  + i)/map.info.resolution);
                    int map_y = (int)((cy + j)/map.info.resolution);
                    // std::cout<<map_x<<'\t'<<map_y<<'\n';
                    M(map_x, map_y)= M(map_x, map_y)<d?d+=M(map_x, map_y):M(map_x, map_y);


 
                }  
            }
        }
        std::cout<<"updated map"<<'\n';                
 
        save_data(M,"map_ros.txt");

        // std::vector<int8_t> map_data(M.data(), M.data() + M.rows() * M.cols());

        map.data = map_data;
        map_pub.publish(map);
        world_T_map.setOrigin( tf::Vector3(0.0, 0.0, 0.0) );
        tf::Quaternion w_q_m;
        w_q_m.setRPY( 0, 0, 0 );  
        world_T_map.setRotation( w_q_m);
        br.sendTransform(tf::StampedTransform(world_T_map, ros::Time::now(), "world","map" ));
        


        
    } 
    else{

      x = msg_val.at(0);
      y = msg_val.at(1);
      theta = msg_val.at(2);
      world_T_robot.setOrigin( tf::Vector3(x, y, 0.0) );
      tf::Quaternion w_q_r;
      w_q_r.setRPY( 0, 0, theta );  
      world_T_robot.setRotation( w_q_r);
      
      vx = msg_val.at(3);
      vy = msg_val.at(4);
      vtheta = msg_val.at(5);
      curr_odom.at(0) = pow((pow(x,2) + pow(y,2)),0.5);
      curr_odom.at(1) = theta;
      odom.header.stamp = ros::Time::now();
      odom.pose.pose.position.x = x;
      odom.pose.pose.position.y = y;
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
