#include<gaussian_map/robot.h>


// robot::robot(double x, double y, double z, double theta1, double theta2, double theta3) {

// }

robot::robot(const Eigen::Matrix4f& initPose) {
    setPose(initPose);
}

void robot::setPose(const Eigen::Matrix4f& currPose) {
    pose_ = currPose;
}

// void robot::setAngularVel(double w)

// void robot::setLinearVel(double v)

// void robot::setTheta(Eigen::Matrix3f R);

Eigen::Matrix4f robot::getPose() {
    return pose_;
}

// Eigen::Vector3f robot::bodyToWorld(const Eigen::Vector3f& p) {
//     Eigen::Vector4f p_ho = Homogenize(p);
//     Eigen::Vector4d q_ho = pose_*p_ho;
//     return Dehomogenize(q_ho);
// }

Eigen::Matrix<float, 3, Eigen::Dynamic> robot::bodyToWorld(const Eigen::Matrix<float, 3, Eigen::Dynamic>& p) {
    Eigen::Matrix<float, 4, Eigen::Dynamic> p_ho = Homogenize(p);
    return Dehomogenize(pose_*p_ho);
}

Eigen::Vector3f robot::worldToBody(const Eigen::Vector3f& p) {
    Eigen::Matrix4f T_inv = Eigen::Matrix4f::Zero(3,3);
    T_inv(3,3) = 1.0;
    Eigen::Matrix3f R = pose_.topLeftCorner(3,3);
    Eigen::Vector3f tr = pose_.block(0,3,3,1);
    // return R.transpose()*p - R.transpose()*tr*p;
    return R.transpose()*(p - tr);
}

Eigen::Matrix<float, 3, Eigen::Dynamic> robot::worldToBody(const Eigen::Matrix<float, 3, Eigen::Dynamic>& p) {
    Eigen::Matrix<float, 4, Eigen::Dynamic> p_ho = Homogenize(p);
    Eigen::Matrix4f T_inv = Eigen::MatrixXf::Zero(4,4);
    T_inv(3,3) = 1.0;
    Eigen::Matrix3f R = pose_.topLeftCorner(3,3);
    Eigen::Vector3f tr = pose_.block(0,3,3,1);
    T_inv.topLeftCorner(3,3) = R.transpose();
    T_inv.block(0,3,3,1) = -R.transpose()*tr;
    return Dehomogenize(T_inv*p_ho);
}

// void robot::updateMotion()

Eigen::Matrix<float, 4, Eigen::Dynamic> Homogenize(const Eigen::Matrix<float, 3, Eigen::Dynamic>& v) {
    int n = v.cols();
    Eigen::Matrix<float, 4, Eigen::Dynamic> v_ho = Eigen::Matrix<float, 4, Eigen::Dynamic>::Constant(4, n, 1.0);
    v_ho.topRows(3) = v;
    return v_ho;
}


Eigen::Matrix<float, 3, Eigen::Dynamic> Dehomogenize(const Eigen::Matrix<float, 4, Eigen::Dynamic>& v) {
    Eigen::VectorXf scale = v.row(3);
    Eigen::Matrix<float, 3, Eigen::Dynamic> v_de = v.topRows(3)* scale.cwiseInverse().asDiagonal();
    return v_de;
}