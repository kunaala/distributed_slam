#include"multiagent_slam/robot2D.h"
#include<Eigen/Core>
#include<fstream>

robot2D::robot2D(double x, double y, double theta) {
    setPosition(x,y);
    setTheta(theta);
    T_ = Eigen::MatrixXd::Zero(3,3);
    T_(2,2) = 1.0;
    updatePose();
    updateTrajectory();
}
void robot2D::setPosition(double x, double y) {
    x_ = x;
    y_ = y;
}
void robot2D::setAngularVel(double w) {
    w_ = w;
}
void robot2D::setLinearVel(double v) {
    v_ = v;
}
void robot2D::setTheta(double theta) {
    theta_ = theta;
}
void robot2D::updatePose() {
    R_ <<cos(theta_),-sin(theta_),sin(theta_),cos(theta_);
    p_ <<x_,y_;
}
void robot2D::updateTrajectory() {
    T_.block(0,0,2,2) = R_;
    T_.block(0,2,2,1) = p_;
}
Eigen::Vector2d robot2D::getPosition() {
    return p_;
}
Eigen::Vector3d robot2D::Homogenize(const Eigen::Vector2d& v) {
    Eigen::Vector3d v_homo;
    v_homo<<v,1;
    return v_homo;
}
Eigen::Vector4d robot2D::Homogenize(const Eigen::Vector3d& v) {
    Eigen::Vector4d v_homo;
    v_homo<<v,1;
    return v_homo;
}
Eigen::VectorXd robot2D::Homogenize(const Eigen::VectorXd& v) {
    Eigen::VectorXd v_homo;
    v_homo<<v,1;
    return v_homo;
}
Eigen::Vector2d robot2D::Dehomogenize(const Eigen::Vector3d& v) {
    Eigen::Vector2d v_de;
    v_de<<v(Eigen::seq(0,Eigen::last-1))/v(Eigen::last);
    return v_de;
}
Eigen::Vector3d robot2D::Dehomogenize(const Eigen::Vector4d& v) {
    Eigen::Vector3d v_de;
    v_de<<v(Eigen::seq(0,Eigen::last-1))/v(Eigen::last);
    return v_de;
}
Eigen::VectorXd robot2D::Dehomogenize(const Eigen::VectorXd& v) {
    Eigen::VectorXd v_de;
    v_de<<v(Eigen::seq(0,Eigen::last-1))/v(Eigen::last);
    return v_de;
}
Eigen::Vector2d robot2D::bodyToWorld(const Eigen::Vector2d& p) {
    Eigen::Vector3d p_homo = Homogenize(p);
    Eigen::Vector3d q_homo = T_*p_homo;
    return Dehomogenize(q_homo);
}
// Eigen::Vector3d robot2D::bodyToWorld(const Eigen::Vector3d& p) {
//     Eigen::Vector4d p_homo = Homogenize(p);
//     Eigen::Vector4d q_homo = T_*p_homo;
//     return Dehomogenize(q_homo);
// }
// Eigen::VectorXd robot2D::bodyToWorld(const Eigen::VectorXd& p) {
//     Eigen::VectorXd p_homo = Homogenize(p);
//     Eigen::VectorXd q_homo = T_*p_homo;
//     return Dehomogenize(q_homo);
// }
Eigen::Vector2d robot2D::worldToBody(const Eigen::Vector2d& p) {
    Eigen::Vector3d p_homo = Homogenize(p);
    Eigen::MatrixXd T_inv_ = Eigen::MatrixXd::Zero(3,3);
    T_inv_(2,2) = 1.0;
    T_.block(0,0,2,2) = R_.transpose();
    T_.block(0,2,2,1) = -R_.transpose()*p_;
    Eigen::Vector3d q_homo = T_inv_*p_homo;
    return Dehomogenize(q_homo);
}
// Eigen::Vector3d robot2D::worldToBody(const Eigen::Vector3d& p) {
//     Eigen::Vector3d p_homo = Homogenize(p);
//     Eigen::MatrixXd T_inv_ = Eigen::MatrixXd::Zero(3,3);
//     T_inv_(2,2) = 1.0;
//     T_.block(0,0,2,2) = R_.transpose();
//     T_.block(0,2,2,1) = -R_.transpose()*p_;
//     Eigen::Vector3d q_homo = T_inv_*p_homo;
//     return Dehomogenize(q_homo);
// }
// Eigen::VectorXd robot2D::worldToBody(const Eigen::VectorXd& p) {
//     Eigen::Vector3d p_homo = Homogenize(p);
//     Eigen::MatrixXd T_inv_ = Eigen::MatrixXd::Zero(3,3);
//     T_inv_(2,2) = 1.0;
//     T_.block(0,0,2,2) = R_.transpose();
//     T_.block(0,2,2,1) = -R_.transpose()*p_;
//     Eigen::Vector3d q_homo = T_inv_*p_homo;
//     return Dehomogenize(q_homo);
// }