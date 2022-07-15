#ifndef _SPARSEGP_
#define _SPARSEGP_
#include<iostream>
#include<random>
#include<Eigen/Dense>
#include <fstream>
#include <cmath>

class SparseGp{
    public:
        SparseGp();
        SparseGp(std::string fname);
        SparseGp(unsigned int dim, unsigned int train_size, unsigned int pseudo_size, 
                     unsigned int test_size, std::pair<int,int> limit, std::string fname);
        std::vector<Eigen::MatrixXf> gen_data();
        void save_data(Eigen::MatrixXf M, std::string filename );
        void posterior(std::vector<Eigen::MatrixXf> &D);
        void posterior(std::vector<Eigen::MatrixXf> &D, Eigen::VectorXf m);


        std::string fname_ = "plot.csv";
        unsigned int dim_=2, train_size_, pseudo_size_, test_size_;
        std::pair<int,int> limit_;

        unsigned int sigma_f_=1,l_=1;




    private:
        Eigen::MatrixXf gen_samples(std::pair<int,int> limit, const int size_,const int dim_);
        Eigen::MatrixXf gen_pseudo_pts(const std::pair<int,int> limit, const unsigned int size, const unsigned int dim);

        Eigen::MatrixXf kernel(Eigen::MatrixXf X1,Eigen::MatrixXf X2);
        

};
#endif