#include"gaussian_map/gpModel.h"
#include <cmath>
#include<iostream>
#include<Eigen/Core>
#include<Eigen/QR>
#include<Eigen/SparseCholesky>
#include <Eigen/Dense>
#include <fstream>

gpModel::gpModel(double c, double l, double sigma) {
    c_ = c;
    l_ = l;
    sigma_ = sigma;
}

void gpModel::set_training_data(Eigen::MatrixXd X, Eigen::VectorXd Y) {
    X_ = X;
    data_size_ = X_.rows();
    y_ = Y;
}

void gpModel::set_pseudo_data(Eigen::MatrixXd X) {
    X_bar_ = X;
    pseudo_data_size_ = X_bar_.rows();
}

void gpModel::set_test_data(Eigen::MatrixXd X) {
    X_test_ = X;
}

Eigen::MatrixXd gpModel::calc_kernel(Eigen::MatrixXd X1, Eigen::MatrixXd X2) {
    Eigen::MatrixXd dist = X1.rowwise().squaredNorm()*Eigen::MatrixXd::Ones(1,X2.rows());
    dist += Eigen::MatrixXd::Ones(X1.rows(), 1) * X2.rowwise().squaredNorm().transpose();
    dist -= 2*(X1 * X2.transpose());
    return c_*((-0.5*dist/pow(l_,2)).array().exp());
}

// void gpModel::train(int iters) {
//     //TODO - hyperparameter training using Gradient ascent.
// }

std::tuple<Eigen::VectorXd,Eigen::VectorXd> gpModel::predict() {
    // std::ofstream dataFile;
    
    Knm_ = calc_kernel(X_,X_bar_);
    Knn_ = calc_kernel(X_,X_);
    Kmm_ = calc_kernel(X_bar_,X_bar_);
    Kmn_ = Knm_.transpose();

    Eigen::LLT<Eigen::MatrixXd> L_Kmm;
    L_Kmm.compute(Kmm_);
    // Kmm_inv_ = Kmm_.completeOrthogonalDecomposition().pseudoInverse();
    Eigen::VectorXd lambda(data_size_);
    for(int i=0;i<data_size_;i++) {
        // lambda(i) = Knn_(i,i) - Knm_.row(i)*Kmm_inv_*(Knm_.row(i).transpose());
        lambda(i) = Knn_(i,i) - Knm_.row(i)*L_Kmm.solve(Knm_.row(i).transpose());
    }
    Eigen::MatrixXd lambda_mat = lambda.asDiagonal();
    Eigen::MatrixXd P = lambda_mat + pow(sigma_,2)*Eigen::MatrixXd::Identity(data_size_,data_size_);
    
    Eigen::LLT<Eigen::MatrixXd> L_P;
    L_P.compute(P);
    // Eigen::MatrixXd P_inv = P.completeOrthogonalDecomposition().pseudoInverse();
    
    Eigen::MatrixXd Qm = Kmm_ + (Knm_.transpose())*L_P.solve(Knm_);
    // Eigen::MatrixXd Qm = Kmm_ + (Knm_.transpose())*P_inv*Knm_;
    // std::cout<<Qm<<"\n";
    Eigen::LLT<Eigen::MatrixXd> L_Qm;
    L_Qm.compute(Qm);

    Kqq_ = calc_kernel(X_test_,X_test_);
    Kqm_ = calc_kernel(X_test_,X_bar_);
    
    mu_pred_ = Kqm_*L_Qm.solve(Kmn_)*L_P.solve(y_);
    
    // mu_pred_ = Kqm_*Qm_inv*Kmn_*P_inv*y_;
    // std::cout<<mu_pred_<<"\n";
    sigma_pred_ = Kqq_.diagonal() - (Kqm_*(L_Kmm.solve(Kqm_.transpose()) - L_Qm.solve(Kqm_.transpose()))).diagonal() + Eigen::VectorXd::Ones(X_test_.rows(),1)*pow(sigma_,2);
    // sigma_pred_ = Kqq_.diagonal() - (Kqm_*(Kmm_inv_ - Qm_inv)*(Kqm_.transpose())).diagonal() + Eigen::VectorXd::Ones(X_test_.rows(),1)*pow(sigma_,2);
    return {mu_pred_,sigma_pred_};
}