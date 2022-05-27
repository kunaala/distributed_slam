#pragma once
#include <Eigen/Core>

class gpModel {
    public:
        gpModel(double c, double l, double sigma);
        void set_training_data(Eigen::MatrixXd X, Eigen::VectorXd Y);
        void set_pseudo_data(Eigen::MatrixXd X);
        void set_test_data(Eigen::MatrixXd X);
        Eigen::MatrixXd calc_kernel(Eigen::MatrixXd X1, Eigen::MatrixXd X2);
        void train(int iters);
        std::tuple<Eigen::VectorXd,Eigen::VectorXd> predict();

    protected:
        Eigen::MatrixXd X_; //Training points
        Eigen::VectorXd y_; //Training points
        Eigen::MatrixXd X_bar_; //Pseudo points
        Eigen::MatrixXd X_test_; //Test points for prediction
        double c_; // Scale hyperparameter of the Kernel
        double l_; // length hyperparameter of the Kernel
        double sigma_; // Input Noise
        Eigen::MatrixXd Knn_;
        Eigen::MatrixXd Knm_;
        Eigen::MatrixXd Kmn_;
        Eigen::MatrixXd Kmm_;
        Eigen::MatrixXd Kmm_inv_;
        Eigen::MatrixXd Kqq_;
        Eigen::MatrixXd Kqm_;
        Eigen::MatrixXd Kqn_;
        int data_size_; //Training data size
        int pseudo_data_size_; //Pseudo data size
        Eigen::VectorXd mu_pred_; //Predicted mean for test values
        Eigen::VectorXd sigma_pred_; //Predicted variance for test values
};