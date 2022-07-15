#include "gaussian_map/MapPub.hpp"
Gaussian_pub::Gaussian_pub(const ros::NodeHandle &nh) : nh_(nh){
    Gaussian_pub::scan_pub = Gaussian_pub::nh_.advertise<sensor_msgs::LaserScan>("scan", 50);
    Gaussian_pub::odom_pub = Gaussian_pub::nh_.advertise<nav_msgs::Odometry>("odom",50);
    Gaussian_pub::map_pub = Gaussian_pub::nh_.advertise<nav_msgs::OccupancyGrid>("map",50);
    double end_angle, start_angle, field_of_view, angular_resolution, maximum_range, laser_frequency;

    Gaussian_pub::nh_.getParam("start_angle", start_angle);
    Gaussian_pub::nh_.getParam("field_of_view", field_of_view);
    Gaussian_pub::nh_.getParam("end_angle", end_angle);
    Gaussian_pub::nh_.getParam("angular_resolution", angular_resolution);
    Gaussian_pub::nh_.getParam("maximum_range", maximum_range);
    Gaussian_pub::nh_.getParam("laser_frequency", laser_frequency);

    Gaussian_pub::scan.header.frame_id = "laser_frame";
    Gaussian_pub::scan.angle_min = start_angle;
    Gaussian_pub::scan.angle_max = (Gaussian_pub::scan.angle_min + field_of_view); 

    Gaussian_pub::scan.angle_increment = angular_resolution; 
    Gaussian_pub::num_readings = (Gaussian_pub::scan.angle_max - Gaussian_pub::scan.angle_min)/Gaussian_pub::scan.angle_increment;
    Gaussian_pub::scan.time_increment = (1 / laser_frequency) / (Gaussian_pub::num_readings);
    Gaussian_pub::scan.range_min = 0.0;
    Gaussian_pub::scan.range_max =maximum_range;
    Gaussian_pub::scan.ranges.resize(Gaussian_pub::num_readings -1);
    Gaussian_pub::odom.header.frame_id = "world";
    Gaussian_pub::odom.child_frame_id = "laser_frame";

    Gaussian_pub::map.header.frame_id = "map";
    double map_width, map_height, map_resolution, map_cx, map_cy, map_cz;

    Gaussian_pub::nh_.getParam("map_width", map_width);
    Gaussian_pub::nh_.getParam("map_height",map_height );
    Gaussian_pub::nh_.getParam("map_resolution", map_resolution );
    Gaussian_pub::nh_.getParam("map_cx", map_cx);
    Gaussian_pub::nh_.getParam("map_cy", map_cy);
    Gaussian_pub::nh_.getParam("map_cz", map_cz);
    Gaussian_pub::map.info.width =  map_width;
    Gaussian_pub::map.info.height = map_height;
    Gaussian_pub::map.info.resolution = map_resolution;
    Gaussian_pub::map.info.origin.position.x = map_cx;
    Gaussian_pub::map.info.origin.position.y = map_cy;
    Gaussian_pub::map.info.origin.position.z = map_cz;
    Gaussian_pub::map.info.origin.orientation = tf::createQuaternionMsgFromYaw(0);
    Gaussian_pub::world_T_map.setOrigin( tf::Vector3(0.0, 0.0, 0.0) );
    std::cout<<"constructed ros viz object"<<'\n';


}


void Gaussian_pub::update_odom(std::vector<double> pose){
    Gaussian_pub::odom.header.stamp = ros::Time::now();
    Gaussian_pub::odom.pose.pose.position.x = pose.at(0)/Gaussian_pub::map.info.resolution;
    Gaussian_pub::odom.pose.pose.position.y = pose.at(1)/Gaussian_pub::map.info.resolution;
    Gaussian_pub::odom.pose.pose.position.z = 0.0/Gaussian_pub::map.info.resolution;
    Gaussian_pub::odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(pose.at(2));
    //set the velocity
    Gaussian_pub::odom.twist.twist.linear.x = pose.at(3);
    Gaussian_pub::odom.twist.twist.linear.y = pose.at(4);
    Gaussian_pub::odom.twist.twist.angular.z = pose.at(5);
    //publish the message
    Gaussian_pub::odom_pub.publish(odom);
    std::cout<<"Published odom"<<'\n';
    Gaussian_pub::world_T_robot.setOrigin( tf::Vector3(odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z) );
    tf::Quaternion w_q_r;
    w_q_r.setRPY( 0, 0, pose.at(2));  
    Gaussian_pub::world_T_robot.setRotation( w_q_r);
    Gaussian_pub::br.sendTransform(tf::StampedTransform(world_T_robot, ros::Time::now(), "world","laser_frame" ));
    // Gaussian_pub::timer_odom_ = Gaussian_pub::nh_.createTimer(ros::Duration(0.1),&Gaussian_pub::odom_callback,this);

}

// void Gaussian_pub::odom_callback(const ros::TimerEvent &event){
//     std::cout<<"Published odom"<<'\n';
//     Gaussian_pub::odom_pub.publish(odom);
// }

void Gaussian_pub::update_map(std::vector<int8_t> rer){
    Gaussian_pub::map.data = rer;
    ros::Rate r(50.0);

    Gaussian_pub::map_pub.publish(Gaussian_pub::map);
    Gaussian_pub::world_T_map.setOrigin( tf::Vector3(0.0, 0.0, 0.0) );
    tf::Quaternion w_q_m;
    w_q_m.setRPY( 0, 0, 0);  
    Gaussian_pub::world_T_map.setRotation( w_q_m);
    Gaussian_pub::br.sendTransform(tf::StampedTransform(world_T_map, ros::Time::now(), "world","map" ));
    // Gaussian_pub::timer_map_ = Gaussian_pub::nh_.createTimer(ros::Duration(0.1),&Gaussian_pub::map_callback,this);
    std::cout<<"Updated map"<<'\n';

    r.sleep();

}

// void Gaussian_pub::map_callback(const ros::TimerEvent &event){
//     std::cout<<"Published map"<<'\n';
//     Gaussian_pub::map_pub.publish(Gaussian_pub::map);

// }

