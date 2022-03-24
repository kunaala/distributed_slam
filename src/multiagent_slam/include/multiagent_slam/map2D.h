#pragma once
#include<iostream>
#include <iterator>
#include <map>
#include<vector>
#include <Eigen/Core>

class map2D {
    protected:
        double x_; //Origin x coordinate
        double y_; //Origin y coordinate
        double resolution_; //Resolution of map. Block size
        double xLimit_; // Limit of map on either side of origin (x_ (+ or -) xLimit)
        double yLimit_; //Limit of map on wither side of origin (y_ (+ or -) yLimit)
        double xMax = 0;
        double yMax = 0;
        double yMin = 0;
        std::map<std::pair<double,double>,double> map;
        std::map<std::pair<double,double>,int> count;
        std::vector<std::vector<double>> X;
        std::vector<double> Y;
        int ll=0;
        int num_pseudo_pts_ = 0;
        int num_training_pts_ = 0;
        // int dimension_;

    public:
        map2D(double x, double y, double resolution, double xLimit, double yLimit);
        map2D(double x, double y, double resolution);
        void set_limits(double xLimit, double yLimit);
        std::pair<int,int> getClosestBlock(double x, double y);
        void setValue(double x, double y, double value);
        double getValue(int x, int y);
        std::vector<int8_t> getGrid();
        std::pair<Eigen::MatrixXd,Eigen::VectorXd> getPseudoPts();
        std::pair<Eigen::MatrixXd,Eigen::VectorXd> getTrainingPts();
        void dumpToFile(std::string fileName);
        double euclidDist(double x1, double y1, double x2, double y2);
};