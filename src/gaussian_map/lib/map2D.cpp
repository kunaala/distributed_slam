#include"gaussian_map/map2D.h"
#include<iostream>
#include<fstream>
#include<Eigen/Core>

map2D::map2D(double x, double y, double resolution, double xLimit, double yLimit) {
    x_ = x;
    y_ = y;
    resolution_ = resolution;
    xLimit_ = xLimit;
    yLimit_ = yLimit;
}

map2D::map2D(double x, double y, double resolution) {
    x_ = x;
    y_ = y;
    resolution_ = resolution;
}

void map2D::set_limits(double xLimit, double yLimit) {
    xLimit_ = xLimit;
    yLimit_ = yLimit;
}


double map2D::euclidDist(double x1, double y1, double x2, double y2) {
    return pow(x1-x2,2)+pow(y1-y2,2);
}

std::pair<int,int> map2D::getClosestBlock(double x, double y) {
    //Compares the distance between the 4 closest points and returns the closest point in the Map.
    int signX = x>=0?1:-1;
    int signY = y>=0?1:-1;
    double p1x = x;
    double p1y = y;
    double p2x = p1x+signX;
    double p2y = p1y;
    double p3x = p1x;
    double p3y = p1y+signY;
    double p4x = p1x+signX;
    double p4y = p1y+signY;
    double d1 = euclidDist(x,y,x_+p1x*resolution_,y_+p1y*resolution_),d2 = euclidDist(x,y,x_+p2x*resolution_,y_+p2y*resolution_);
    double d3 = euclidDist(x,y,x_+p3x*resolution_,y_+p3y*resolution_),d4 = euclidDist(x,y,x_+p4x*resolution_,y_+p4y*resolution_);
    if(d1>d2) {
        if(d2>d3) {
            if(d3>d4) return {p4x,p4y};
            else return {p3x,p3y};
        }
        else {
            if(d2>d4) return {p4x,p4y};
            else return {p2x,p2y};
        }
    }
    else {
        if(d1>d3) {
            if(d3>d4) return {p4x,p4y};
            else return {p3x,p3y};
        }
        else {
            if(d1>d4) return {p4x,p4y};
            else return {p1x,p1y};
        }
    }
}

void map2D::setValue(double x, double y, double value) {
    std::pair<double,double> pt = getClosestBlock(x/resolution_,y/resolution_);
    num_training_pts_++;
    X.push_back({x/resolution_,y/resolution_});
    Y.push_back(value);
    if(map.find(pt)==map.end()) {
        map.insert({pt,value});
        count.insert({pt,1});
        num_pseudo_pts_++;
    }
    else {
        // if(value<0) {
        //     map[pt] = std::min(map[pt],value);
        yMin = std::max(yMin,map[pt]);
        // }
        // else {
        //     map[pt] = std::max(map[pt],value);
        yMax = std::max(yMax,map[pt]);
        // }
        count[pt]+=1;
        map[pt]+= (map[pt]-value)/count[pt];
        // X.push_back({x,y});
        // Y.push_back(value);
    }
}

double map2D::getValue(int x, int y) {
    std::pair<double,double> pt(x,y);
    if(map.find(pt)==map.end()) {
        return 701.0;
    }
    else return map[pt]/count[pt];
}

std::pair<Eigen::MatrixXd,Eigen::VectorXd> map2D::getPseudoPts() {
    std::cout<<num_pseudo_pts_<<"\n";
    Eigen::MatrixXd xps(num_pseudo_pts_,2);
    Eigen::VectorXd yps(num_pseudo_pts_);
    int i = 0;
    for(auto const& x : map) {
        xps(i,0) = x.first.first;
        xps(i,1) = x.first.second;
        yps(i) = x.second;
        i++;
    }
    return {xps,yps};
}

std::pair<Eigen::MatrixXd,Eigen::VectorXd> map2D::getTrainingPts() {
    std::cout<<num_training_pts_<<"\n";
    Eigen::MatrixXd xtr(num_training_pts_,2);
    Eigen::VectorXd ytr(num_training_pts_);
    for(int i=0;i<num_training_pts_;i++) {
        xtr(i,0) = X[i][0];
        xtr(i,1) = X[i][1];
        ytr(i) = Y[i];
    }
    return {xtr,ytr};
}

std::vector<int8_t> map2D::getGrid() {
    std::vector<int8_t> grr(6000*6000,-1);
    int numX = xLimit_/resolution_;
    int numY = yLimit_/resolution_;
    for(auto const& x : map) {
        grr[int(x.first.first + numX)*6000 + int(x.first.second+numY)] = int8_t(100*(x.second-yMin)/(yMax-yMin));
    }
    return grr;
}

void map2D::dumpToFile(std::string fileName) {
    std::ofstream dataFile;
    dataFile.open(fileName);
    int numX = xLimit_/resolution_;
    int numY = yLimit_/resolution_;
    for(int i=0;i<2*numY;i++) {
        for(int j=0;j<2*numX;j++) {
            dataFile << getValue(-numX+j,numY-i)<<",";
        }
        dataFile<<"\n";
    }
    dataFile.close();
}
