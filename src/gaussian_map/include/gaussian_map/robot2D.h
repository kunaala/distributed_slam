#ifndef ROBOT2D_H
#define ROBOT2D_H
#include<Eigen/Core>

class robot2D {
    protected:
        double x_; //X - position
        double y_; //Y - position
        double theta_; //Angle - Orientation
        double v_; //Velocity
        double w_; //Angular velocity
        double accel_; //Acceleration
        Eigen::Matrix2d R_; //Rotation wrt world frame
        Eigen::Vector2d p_; //Position wrt world frame
        Eigen::Matrix3d T_; //Trajectory matrix - [R|p];[0|1]
        Eigen::Matrix3d T_inv_; //Inverse of Trajectory matrix
    
    public:
        robot2D(double x, double y, double theta);
        void setPosition(double x, double y);
        void setAngularVel(double w);
        void setLinearVel(double v);
        void setTheta(double theta);
        void updatePose();
        void updateTrajectory();
        Eigen::Vector2d getPosition();
        Eigen::Vector3d Homogenize(const Eigen::Vector2d& v);
        Eigen::Vector4d Homogenize(const Eigen::Vector3d& v);
        Eigen::VectorXd Homogenize(const Eigen::VectorXd& v);
        Eigen::Vector2d Dehomogenize(const Eigen::Vector3d& v);
        Eigen::Vector3d Dehomogenize(const Eigen::Vector4d& v);
        Eigen::VectorXd Dehomogenize(const Eigen::VectorXd& v);
        Eigen::Vector2d bodyToWorld(const Eigen::Vector2d& p);
        Eigen::Vector3d bodyToWorld(const Eigen::Vector3d& p);
        Eigen::VectorXd bodyToWorld(const Eigen::VectorXd& p);
        Eigen::Vector2d worldToBody(const Eigen::Vector2d& p);
        Eigen::Vector3d worldToBody(const Eigen::Vector3d& p);
        Eigen::VectorXd worldToBody(const Eigen::VectorXd& p);
};
#endif 