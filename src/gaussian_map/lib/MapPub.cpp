#include "gaussian_map/MapPub.hpp"
MapPub::MapPub(ros::NodeHandle &nh) : nh_(nh){

    MapPub::scan_pub = MapPub::nh_.advertise<sensor_msgs::LaserScan>("scan", 50);
    MapPub::odom_pub = MapPub::nh_.advertise<nav_msgs::Odometry>("odom",50);
    MapPub::map_pub = MapPub::nh_.advertise<nav_msgs::OccupancyGrid>("map",50);
    double end_angle, start_angle, field_of_view, angular_resolution, maximum_range, laser_frequency;

    MapPub::nh_.getParam("start_angle", start_angle);
    MapPub::nh_.getParam("field_of_view", field_of_view);
    MapPub::nh_.getParam("end_angle", end_angle);
    MapPub::nh_.getParam("angular_resolution", angular_resolution);
    MapPub::nh_.getParam("maximum_range", maximum_range);
    MapPub::nh_.getParam("laser_frequency", laser_frequency);
    

    MapPub::scan.header.frame_id = "lidar";
    MapPub::scan.angle_min = start_angle;
    MapPub::scan.angle_max = (MapPub::scan.angle_min + field_of_view); 
    MapPub::scan.angle_increment = angular_resolution; 

    MapPub::num_readings = (MapPub::scan.angle_max - MapPub::scan.angle_min)/MapPub::scan.angle_increment;
    MapPub::scan.time_increment = (1 / laser_frequency) / (MapPub::num_readings);
    MapPub::scan.range_min = 0.0;

    MapPub::scan.range_max =maximum_range;
    
    MapPub::scan.ranges.resize(MapPub::num_readings-1);

    MapPub::odom.header.frame_id = "world";
    MapPub::odom.child_frame_id = "robot";

    MapPub::map.header.frame_id = "map";
    double map_width, map_height, map_resolution, map_cx, map_cy, map_cz;

    MapPub::nh_.getParam("map_width", map_width);
    MapPub::nh_.getParam("map_height",map_height );
    MapPub::nh_.getParam("map_resolution", map_resolution );
    MapPub::nh_.getParam("map_cx", map_cx);
    MapPub::nh_.getParam("map_cy", map_cy);
    MapPub::nh_.getParam("map_cz", map_cz);
    MapPub::map.info.width =  map_width;
    MapPub::map.info.height = map_height;
    MapPub::map.info.resolution = map_resolution;
    MapPub::map.info.origin.position.x = map_cx;
    MapPub::map.info.origin.position.y = map_cy;
    MapPub::map.info.origin.position.z = map_cz;
    MapPub::map.info.origin.orientation = rpy_to_q(0,0,0);


}


void MapPub::update_odom(Eigen::VectorXf pose){
    MapPub::odom.header.stamp = ros::Time::now();
    MapPub::odom.pose.pose.position.x = pose[0]/MapPub::map.info.resolution;
    MapPub::odom.pose.pose.position.y = pose[1]/MapPub::map.info.resolution;
    MapPub::odom.pose.pose.position.z = 0.0/MapPub::map.info.resolution;
    geometry_msgs::Quaternion w_q_r = rpy_to_q(0,0,pose[2]);
    MapPub::odom.pose.pose.orientation = w_q_r;
    //set the velocity
    // MapPub::odom.twist.twist.linear.x = pose[3];
    // MapPub::odom.twist.twist.linear.y = pose[4];
    // MapPub::odom.twist.twist.angular.z = pose[5];
    MapPub::odom.twist.twist.linear.x = 0;
    MapPub::odom.twist.twist.linear.y = 0;
    MapPub::odom.twist.twist.angular.z = 0;
    //publish the message
    MapPub::odom_pub.publish(odom);
    std::cout<<"Published odom"<<'\n';

    //For updating in Map
    MapPub::world_T_robot.header.stamp = ros::Time::now();
	MapPub::world_T_robot.header.frame_id = "world";
	MapPub::world_T_robot.child_frame_id = "robot";
	MapPub::world_T_robot.transform.translation.x = pose[0];
	MapPub::world_T_robot.transform.translation.y = pose[1];
	MapPub::world_T_robot.transform.translation.z = 0.0;
	
	MapPub::world_T_robot.transform.rotation.x = w_q_r.x;
	MapPub::world_T_robot.transform.rotation.y = w_q_r.y;
	MapPub::world_T_robot.transform.rotation.z = w_q_r.z;
	MapPub::world_T_robot.transform.rotation.w = w_q_r.w;
	
    MapPub::br_.sendTransform(MapPub::world_T_robot);

}


void MapPub::update_map(std::vector<int8_t> rer){
    MapPub::map.data = rer;
    ros::Rate r(50.0);

    MapPub::map_pub.publish(MapPub::map);
    MapPub::world_T_map.header.stamp = ros::Time::now();
	MapPub::world_T_map.header.frame_id = "world";
	MapPub::world_T_map.child_frame_id = "map";
	MapPub::world_T_map.transform.translation.x = 0.0;
	MapPub::world_T_map.transform.translation.y = 0.0;
	MapPub::world_T_map.transform.translation.z = 0.0;
	geometry_msgs::Quaternion w_q_m = rpy_to_q(0,0,0);
	MapPub::world_T_map.transform.rotation.x = w_q_m.x;
	MapPub::world_T_map.transform.rotation.y = w_q_m.y;
	MapPub::world_T_map.transform.rotation.z = w_q_m.z;
	MapPub::world_T_map.transform.rotation.w = w_q_m.w;
	
    MapPub::br_.sendTransform(MapPub::world_T_map);
    
    std::cout<<"Updated map"<<'\n';

    r.sleep();

}



