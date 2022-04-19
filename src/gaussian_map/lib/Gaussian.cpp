#include "Gaussian.hpp"

Gaussian::Gaussian(const ros::Nodehandle &nh): nh_(nh){
    Gaussian::scan_pub_ = nh_.advertise<sensor_msgs::LaserScan>("scan", 50);
    Gaussian::odom_pub_ = nh_.advertise<nav_msgs::Odometry>("odom",50);
    Gaussian::map_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("map",50);
    Gaussian::timer_ = nh_.createTimer(ros::Duration(0.01), &Gaussian::timer_callback, this); 
    Gaussian::param_map["start_angle"] = -1.570796;
    Gaussian::param_map["field_of_view"] = 3.141593;
    Gaussian::param_map["angular_resolution"] = 0.017453294;
    Gaussian::param_map["maximum_range"] = 81.920000;
    Gaussian::param_map["laser_frequency"] = 40;

    scan.header.frame_id = "laser_frame";
    scan.angle_min = param_map["start_angle"] + curr_odom.at(1); // pose correction
    scan.angle_max = (scan.angle_min + param_map["field_of_view"]) ; 
    scan.angle_increment = param_map["angular_resolution"] ; 
    unsigned int num_readings = (scan.angle_max - scan.angle_min)/scan.angle_increment;
    scan.time_increment = (1 / param_map["laser_frequency"]) / (num_readings);
    scan.range_min = 0.0;
    scan.range_max = param_map["maximum_range"];

    map.info.width =500;
    map.info.height = 500;
    map.info.resolution = 0.1;
    map.header.frame_id = "map";
    map.info.origin.position.x = -25;
    map.info.origin.position.y = -25;
    map.info.origin.position.z = 0.0;
}

double Gaussian::tsdf(double p1_x, double p1_y, double p2_x, double p2_y, double qx, double qy){
    double d = abs((p2_x - p1_x) * (p1_y- qy) - (p1_x - qx)*(p2_y - p1_y));
    d /= sqrt(pow((p2_x - p1_x),2) + pow((p2_y - p1_y),2));
    return d;
}
double Gaussian::scale(double z, double resol){
  z/=resol;
  return z;

}

std::vector<std::string> Gaussian::split_line(std::string str){
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

std::vector<double> Gaussian::slice_data(std::string str){
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

MatrixXd Gaussian::polar_cart(MatrixXd r, MatrixXd theta){
    // MatrixXd z(4,r.cols());
    MatrixXd z(2,r.cols());

    z.row(0) = (r.array() * theta.array().cos()).matrix();
    z.row(1) = (r.array() * theta.array().sin()).matrix();
    // z.row(2) = VectorXd::Zero(r.cols());
    // z.row(3) = VectorXd::Ones(r.cols());
    return z;
}

void Gaussian::timer_callback(const ros::TimerEvent &event){

}


void Gaussian::save_data(MatrixXd m, std::string filename ){

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