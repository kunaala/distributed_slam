
#include<iostream>
#include<Eigen/Core>
#include<fstream>
#include<random>
#include<math.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include"multiagent_slam/robot2D.h"
#include"multiagent_slam/map2D.h"
#include "multiagent_slam/gpModel.h"


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

std::vector<int8_t> getDistGrid(Eigen::MatrixXd X_test, Eigen::VectorXd Y_mean ,double resolution, double xLimit, double yLimit) {
    std::vector<int8_t> grr(6000*6000,-1);
    int numX = xLimit/resolution;
    int numY = yLimit/resolution;
    int num = X_test.rows();
    std::cout<<num<<"\n";
    double yMin = 0.0, yMax = 0.0;
    for(int i=0;i<num;i++) {
        yMin = std::min(yMin,Y_mean(i));
        yMax = std::max(yMax,Y_mean(i));
    }
    std::cout<<yMin<<" "<<yMax<<"\n";
    for(int i=0;i<num;i++) {
        grr[int(X_test(i,0) + numX)*6000 + int(X_test(i,1) + numY)] = int8_t(100*(Y_mean(i) - yMin)/(yMax-yMin));
    }
    return grr;
}

int main(int argc, char *argv[] ) {

    ros::init(argc, argv, "laser_scan_publisher");
    ros::NodeHandle n;
    ros::Publisher scan_pub = n.advertise<sensor_msgs::LaserScan>("scan", 10,true);
    ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom",10,true);
    ros::Publisher map_pub = n.advertise<nav_msgs::OccupancyGrid>("map",10,true);
    ros::Rate r(50.0);
 
    // std::cout<<map_pub.get_num_connections()<<

    nav_msgs::Odometry odom;
    odom.header.frame_id = "world";
    odom.child_frame_id = "laser_frame";
    double x,y,theta,vx,vy,vtheta;

    



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
    world_T_map.setOrigin( tf::Vector3(0.0, 0.0, 0.0) );

   


    double x_orig=0.0,y_orig=0.0,theta_orig=0.0;
    double grid_res = 0.1;
    robot2D Mace(0.0,0.0,theta_orig);
    double xL = 300.0, yL = 300.0;
    map2D theMap(0,0,grid_res,xL,yL);
    // map2D realMap(0,0,grid_res,300,300);
    std::ifstream inpFile ("intel.gfs.log");
    std::string line;
    size_t pos = 0;
    double theta_initial = -1.570796;
    double angular_res = 2*0.008727;
    int idx = 0;

    std::vector<double> msg_val;
    sensor_msgs::LaserScan scan;
    scan.header.frame_id = "laser_frame";
    scan.angle_min = theta_initial + theta; // pose correction
    scan.angle_max = (scan.angle_min + 1.5707) ; 
    scan.angle_increment = angular_res; 
    unsigned int num_readings = (scan.angle_max - scan.angle_min)/scan.angle_increment;
    scan.time_increment = (1 / 40) / (180);
    scan.range_min = 0.0;
    scan.range_max =81.920000;
    scan.ranges.resize(179);
    double sigma_noise, l, c, up, down;
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
    std::vector<int8_t> rer(6000*6000,-1);
    // Reading file and compute Pseudo pts and compute distances
    while(ros::ok) {
        // map.data = rer;
        // map_pub.publish(map);
        idx++;
        // if(idx==175) break;
        if(idx%200==0) std::cout<<idx<<"\n";
        // inpFile>>line;
        std::getline(inpFile,line);
        // if(line=="ODOM") std::ofstream test ("testOut");
        if(inpFile.eof()) break;
        std::vector<std::string> input;
        std::string token;
        std::string delimiter = " ";
        while ((pos = line.find(delimiter)) != std::string::npos) {
            token = line.substr(0, pos);
            input.push_back(token);
            line.erase(0, pos + delimiter.length());
        }
        if(input[0]=="ODOM") {
            double x_pose = stod(input[1])-x_orig;
            double y_pose = stod(input[2])-y_orig;
            double theta_pose = stod(input[3]);
            Mace.setPosition(x_pose,y_pose);
            Mace.setTheta(theta_pose);

            world_T_robot.setOrigin( tf::Vector3(x_pose/resol, y_pose/resol, 0.0) );
            tf::Quaternion w_q_r;
            w_q_r.setRPY( 0, 0, theta_pose);  
            world_T_robot.setRotation( w_q_r);

            vx = 0;
            vy = 0;
            vtheta = 0;
            odom.header.stamp = ros::Time::now();
            odom.pose.pose.position.x = x_pose/resol;
            odom.pose.pose.position.y = y_pose/resol;
            odom.pose.pose.position.z = 0.0;
            odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(theta_pose);
            //set the velocity
            odom.twist.twist.linear.x = vx;
            odom.twist.twist.linear.y = vy;
            odom.twist.twist.angular.z = vtheta;

            //publish the message
            odom_pub.publish(odom);
            br.sendTransform(tf::StampedTransform(world_T_robot, ros::Time::now(), "world","laser_frame" ));
        }
        else if(input[0]=="FLASER") {
            ros::Time scan_time = ros::Time::now();
            scan.header.stamp = scan_time;
            Mace.updatePose();
            Mace.updateTrajectory();
            Eigen::Vector2d currPos = Mace.getPosition();
            int num_readings = stoi(input[1]);
            for(int i=1;i<num_readings;i++) {
                double dist = stod(input[1+i]);
                scan.ranges[i-1] = dist;
                // std::cout<<dist<<" ";
                double dist_next = stod(input[1+i+1]);
                double angle = theta_initial + i*angular_res;
                double pt_x1 = cos(angle)*dist, pt_y1 = sin(angle)*dist;
                // std::cout<<pt_x1<<":"<<pt_y1<<"\n";
                // Eigen::Vector2d tt(pt_x1,pt_y1);
                // Eigen::Vector2d correctPt = Mace.bodyToWorld(tt);

                // realMap.setValue(correctPt(0),correctPt(1),1);
                double pt_x2 = cos(angle+angular_res)*dist_next, pt_y2 = sin(angle+angular_res)*dist_next;
                std::vector<std::pair<double,double> > psPts = getPseudoPts(pt_x1,pt_y1,grid_res);
                for(int j=0;j<9;j++) {
                    double gVal = getSignedDist(psPts[j].first,psPts[j].second,pt_x1,pt_y1,pt_x2,pt_y2);
                    if(euclidDist(psPts[j].first,psPts[j].second,0,0)>euclidDist(pt_x1,pt_y1,0,0)) {
                        gVal = -gVal;
                    }
                    Eigen::Vector2d ptBody(psPts[j].first,psPts[j].second);
                    Eigen::Vector2d ptWorld = Mace.bodyToWorld(ptBody);
                    theMap.setValue(ptWorld(0),ptWorld(1),gVal);
                }
            }
            std::pair<Eigen::MatrixXd,Eigen::VectorXd> psData = theMap.getPseudoPts();
            std::pair<Eigen::MatrixXd,Eigen::VectorXd> trData = theMap.getTrainingPts();

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
            // dataFile<<"\n";
            std::tuple<Eigen::VectorXd, Eigen::VectorXd> res = model.predict();

            Eigen::VectorXd mu_pred = std::get<0>(res);
            Eigen::VectorXd sigma_pred = std::get<1>(res);
            // std::ofstream ddf;
            // ddf.open("gp.csv");
            rer = getDistGrid(X_test,mu_pred,grid_res,xL,yL);
            map.data = rer;
            map_pub.publish(map);
            world_T_map.setOrigin( tf::Vector3(0.0, 0.0, 0.0) );
            tf::Quaternion w_q_m;
            w_q_m.setRPY( 0, 0, 0);  
            world_T_map.setRotation( w_q_m);
            br.sendTransform(tf::StampedTransform(world_T_map, ros::Time::now(), "world","map" ));
            std::cout<<"once"<<'\n';
        }
        r.sleep();
        // std::vector<int8_t> tes = theMap.getGrid();
    }
    inpFile.close();
    // theMap.dumpToFile("outDataGrid.csv");
    // realMap.dumpToFile("RealOut.csv");

    // Computing Gaussian distances
    // std::pair<Eigen::MatrixXd,Eigen::VectorXd> psData = theMap.getPseudoPts();
    // std::pair<Eigen::MatrixXd,Eigen::VectorXd> trData = theMap.getTrainingPts();

    // gpModel model(c,l,sigma_noise);
    // model.set_training_data(trData.first,trData.second);
    // model.set_pseudo_data(psData.first);

    // std::random_device rd{};
    // std::mt19937 gen{rd()};
    // std::normal_distribution<> rand{0, 1.0};
    // int temp = psData.second.size();
    // Eigen::MatrixXd X_test(temp,2);

    // for(int i=0;i<temp;i++) {
    // // for(int j=0;j<temp;j++) {
    // //     // std::cout<<i*temp + j<<"\n";
    //     X_test(i,0) = psData.first(i,0) + rand(gen);
    //     X_test(i,1) = psData.first(i,1) + rand(gen);
    // // dataFile << X_test(i,0)<<" "<<X_test(i,1)<<" ";
    // // }
    // }
    // model.set_test_data(X_test);
    // // dataFile<<"\n";
    // std::tuple<Eigen::VectorXd, Eigen::VectorXd> res = model.predict();

    // Eigen::VectorXd mu_pred = std::get<0>(res);
    // Eigen::VectorXd sigma_pred = std::get<1>(res);
    // // std::ofstream ddf;
    // // ddf.open("gp.csv");
    // rer = getDistGrid(X_test,mu_pred,grid_res,xL,yL);
    // map.data = rer;
    // map_pub.publish(map);
    // world_T_map.setOrigin( tf::Vector3(0.0, 0.0, 0.0) );
    // tf::Quaternion w_q_m;
    // w_q_m.setRPY( 0, 0, 0);  
    // world_T_map.setRotation( w_q_m);
    // br.sendTransform(tf::StampedTransform(world_T_map, ros::Time::now(), "world","map" ));
    // std::cout<<"once"<<'\n';
    
    // sleep(10);
    // std::ofstream dataFile;
    // dataFile.open("data.csv");
   
    // std::cout<<"breaking"<<'\n';
// for(int q1=0;q1<6000;q1++) {
    //     for(int q2=0;q2<6000;q2++) {
    //         ddf<<rer[q1*6000 + q2]<<" ";
        // }
        // ddf<<"\n";
    // }
    // ddf.close();
    // for(int i=0;i<temp;i++) {
    //     dataFile<<mu_pred(i)<<",";
    // }
    // dataFile<<"\n";

    // for(int i=0;i<temp;i++) {
    //     dataFile<<sigma_pred(i)<<",";
    // }
    // dataFile<<"\n";
    // dataFile.close();
    return 0;
}