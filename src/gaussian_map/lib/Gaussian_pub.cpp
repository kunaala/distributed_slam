#include "gaussian_map/Gaussian_pub.hpp"
Gaussian_pub(ros::Nodehandle &nh) : Gaussian_pub::nh(nh){
    ros::Publisher scan_pub = Gaussian_pub::nh.advertise<sensor_msgs::LaserScan>("scan", 50);
    ros::Publisher odom_pub = Gaussian_pub::nh.advertise<nav_msgs::Odometry>("odom",50);
    ros::Publisher map_pub = Gaussian_pub::nh.advertise<nav_msgs::OccupancyGrid>("map",50);
    Gaussian_pub::nh.getParam("start_angle", start_angle);
    Gaussian_pub::nh.getParam("field_of_view", field_of_view);
    Gaussian_pub::nh.getParam("end_angle", end_angle);
    Gaussian_pub::nh.getParam("angular_resolution", angular_resolution);
    Gaussian_pub::nh.getParam("maximum_range", maximum_range);
    Gaussian_pub::nh.getParam("laser_frequency", laser_frequency);
    Gaussian_pub::nh.getParam("log_file", Gaussian_pub::data_file);
    std::ifstream f(Gaussian_pub::data_file,std::ios::in);
    Gaussian_pub::filename = f;

    Gaussian_pub::odom.header.frame_id = "world";
    Gaussian_pub::odom.child_frame_id = "laser_frame";

    Gaussian_pub::map.info.width =6000;
    Gaussian_pub::map.info.height = 6000;
    Gaussian_pub::map.info.resolution = 0.1;
    double resol = 0.1;
    Gaussian_pub::map.header.frame_id = "map";
    Gaussian_pub::map.info.origin.position.x = -300;
    Gaussian_pub::map.info.origin.position.y = -300;
    Gaussian_pub::map.info.origin.position.z = 0.0;
    Gaussian_pub::map.info.origin.orientation = tf::createQuaternionMsgFromYaw(0);
    Gaussian_pub::world_T_map.setOrigin( tf::Vector3(0.0, 0.0, 0.0) );

}
