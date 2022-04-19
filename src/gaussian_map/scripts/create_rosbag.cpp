#define _USE_MATH_DEFINES
#include <cmath>
#include <iostream>
#include <fstream>
#include <vector>
#include <Eigen/Dense>
//#include "ros/ros.h"
//#include "rosbag/bag.h"
//#include "sensor_msgs/LaserScan.h"
//#include "std_msgs/Float64MultiArray.h"
//#include "std_msgs/MultiArrayDimension.h"
//#include "std_msgs/MultiArrayLayout.h"

float scale_offset(unsigned short x){
	float scale_factor = 0.005; //5mm
	int offset = -100;
	double new_x(x * scale_factor + offset);
	return new_x;
}
int main(int argc, char *argv[]) {
	// handling commandline arguments
	if (argc < 2 ) {
		std::cout<<"Expecting hokuyo_30 file name:"<<'\n';
		return 1;
	}
	std::string filename;
	if (argc < 3 ) filename= "lidar_dataset.bag";
	else filename= argv[argc-1];
	// handling commandline arguments
	
	const int num_hits= 1081;
	const double rad0 = -135 * (M_PI/180.0);
    	const double radstep = 0.25 * (M_PI/180.0);
	Eigen::Vector<double, num_hits> angles;
	for(unsigned int i = 0 ; i <  num_hits ; i++) {
		// Similar to linspace
		angles(i) = rad0 + i * (double(num_hits-1)*radstep / num_hits);
	}
	unsigned long long epoch_time;
	unsigned short s;
	
	//ros::Time timestamp;
	//std_msgs::MultiArrayLayout layout;
	//std_msgs::MultiArrayDimension dim;
	//std_msgs::Float64MultiArray hits;
	// Reading bin file
	std::ifstream file (argv[1], std::ios::in|std::ios::binary);
	//rosbag::Bag bag(filename, rosbag::bagmode::Write);

	if ( file.is_open()) {
		while(!file.eof()) {
			file.read((char *)&epoch_time,8);
			Eigen::Vector<double,num_hits> scan,x,y;

			for(int i=0;i<num_hits;i++){
				file.read((char *)&s, 2);
				scan(i) = scale_offset(s);
				//std::cout<<"scan result"<<scan.at(i)<<'\t';
			}
//			Conversion from  polar coordinates
//			std::cout<<scan<<'\n';
			// x = scan.dot(angles.array().cos());
			// y = scan.dot(angles.array().sin());
			
			//timestamp = ros::TimeBase<ros::Time , ros::Duration>::fromSec(epoch_time/pow(10,6));
			//layout.dim=dim;
			//layout.dim[0].label = "scan";
            		//layout.dim[0].size = num_hits;
            		//layout.dim[0].stride = 1;
			//hits.data = scan;
			//hits.layout = layout;
			
			//bag.write("/scan_packets", ros::Time::now(),hits);
			
		}
	}
	else std::cerr<<"hokuyo file could not be opened"<<'\n';
	file.close();
	//bag.close();
	return 0;	
}
