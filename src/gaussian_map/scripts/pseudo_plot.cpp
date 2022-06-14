#include<iostream>
#include<fstream>
#include<sstream>
#include<vector>
#include<Eigen/Dense>
#include<map>
#include<unordered_map>
#include<cmath>

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

int main(int charc, char *argv[]){

    std::unordered_map<std::string, double> param_map;
    //   unsigned int params = 8;
    //   for(unsigned int i =0;i<params;i++){
    //       param_map[msg_h.at(i)] = msg_val.at(i);
    //   }
    
    param_map["start_angle"] = -1.570796;
    param_map["field_of_view"] = 3.141593;
    param_map["end_angle"] = param_map["start_angle"] + param_map["field_of_view"];
    
    param_map["angular_resolution"] = 0.017453294;
    param_map["num_readings"] = (param_map["end_angle"] - param_map["start_angle"]) / param_map["angular_resolution"] ;
    param_map["maximum_range"] = 81.920000;
    param_map["laser_frequency"] = 40;

    std::unordered_map<std::string, double> map_hyper;
    map_hyper["width"] =50;
    map_hyper["height"]  = 50;
    map_hyper["resol"]  = 0.1;
    map_hyper["c_x"] =  (int)(map_hyper["width"]/2);
    map_hyper["c_y"] = (int)(map_hyper["height"]/2);
    map_hyper["c_z"] = 0.0;
    int size_x= (int)(map_hyper["height"]/map_hyper["resol"]);
    int size_y=   (int)(map_hyper["width"]/map_hyper["resol"]);
    int x_min = -(int)(map_hyper["width"]/2);
    int y_min = -(int)(map_hyper["height"]/2);

    MatrixXd map(size_x,size_y);
    
    MatrixXd angles = VectorXd::LinSpaced(param_map["num_readings"], param_map["start_angle"],param_map["end_angle"] ).transpose() ;
    std::ifstream f("filtered2.log",std::ios::in);
    std::string data;
    double x=0,y=0,theta=0;
    int local_cx = 1,local_cy =1;
    unsigned int msg_id = 0;
    // Matrix4d world_T_robot;
    Matrix2d world_T_robot;
    unsigned int err_count =0;
    while(!f.eof()){
        std::getline(f,data);
        std::vector<double> msg_val;
        msg_val = slice_data(data);

        
        if (msg_val.at(0) == 180){
            /* populate the LaserScan message */
            std::cout<<msg_id<<'\n';
            msg_id++;
            std::vector<double> readings(msg_val.begin()+1, msg_val.end()-7);
            Map<MatrixXd> ranges(readings.data(),1,readings.size());
            for(int r=0;r<readings.size();r++){
                if (ranges(0,r) >80) ranges(0,r) =0; 
                // ranges(0,r) /= map_hyper["resol"];
            }
            MatrixXd z = polar_cart(ranges, angles);
            // std::cout << "shape"<<z.rows() <<"x"<<z.cols()  << '\n';
            z.row(0) = z.row(0) / map_hyper["resol"];
            z.row(1) = z.row(1) / map_hyper["resol"];
            Vector2d p;
            p << x,
                 y;
            MatrixXd zw = (world_T_robot * z).colwise() + p;
            // MatrixXd zw = world_T_robot * z;

            for(int k=0;k<readings.size()-1;k++){
                double p1_x = zw.col(k)(0) ,  p1_y = zw.col(k)(1);
                double p2_x = zw.col(k+1)(0) ,  p2_y = zw.col(k+1)(1);
                local_cx = int(zw.col(k)(0));
                local_cy = int(zw.col(k)(1));
                for(int i=local_cx-1;i<=local_cx+1;i++){
                    for(int j=local_cy-1;j<=local_cy+1;j++){
                        double d = tsdf(p1_x,p1_y,p2_x,p2_y,i,j);
                        int map_x = (int)(map_hyper["c_x"] + i );
                        int map_y = (int)(map_hyper["c_y"] + j );
                        // std::cout<<p2_x<<p2_y<<map_x<<'\t'<<map_y<<'\n';
                        if (map_x >=0 && map_y>=0) map(map_x, map_y)= map(map_x, map_y)<d?d:map(map_x, map_y);
                        else err_count+=1;
                        // map(map_hyper["c_x"] + 0,0) = d;
                    }  
                }
            } 
            
            // for (auto i:local_map) std::cout<<i<<'\t';
            // std::cout<<'\n';
            



        }
        else{
            x = msg_val.at(0);
            y = msg_val.at(1);
            theta = msg_val.at(2);
            double vx = msg_val.at(3);
            double vy = msg_val.at(4);
            double vtheta = msg_val.at(5);

            // world_T_robot << cos(theta), -sin(theta), 0, x,
            //                 sin(theta), cos(theta), 0, y,
            //                 0 ,0, 1, 0,
            //                 0 ,0 ,0, 1;
            world_T_robot << cos(theta), -sin(theta),
                             sin(theta), cos(theta);
        }

    }
    std::cout<<err_count<<'\n';
    save_data(map,"map_gen.txt");



}
  