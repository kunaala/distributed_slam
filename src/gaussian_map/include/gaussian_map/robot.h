#include<Eigen/Core>

class robot {
    protected:
        Eigen::Matrix4f pose_;
    
    public:

        /**
         * Constructor using the initial robot position. Using
         * coordinates seperately
         *
         * \param[in] x X-coordinate.
         * \param[in] y Y-coordinate
         * \param[in] z Z-coordinate
         * \param[in] theta1 Angle wrt x axis
         * \param[in] theta2 Angle wrt y axis
         * \param[in] theta3 Angle wrt z axis
         */
        robot(double x, double y, double z, double theta1, double theta2, double theta3);

        /**
         * Constructor using the initial robot position. Using the
         * pose directly. Prefer this.
         *
         * \param[in] initPose Pose in 4x4 trajectory matrix form.
         */
        robot(const Eigen::Matrix4f& initPose);

        /**
         * Updates current pose of the robot.
         * Takes in Trajectory 4x4 matrix.
         *
         * \param[in] currPose Pose in 4x4 trajectory matrix form.
         */
        void setPose(const Eigen::Matrix4f& currPose);

        /**
         * Updates angular velocity of robot.
         *
         * \param[in] w Scalar angular velocity value in rad/s.
         */
        void setAngularVel(double w);

        /**
         * Updates linear velocity of robot.
         *
         * \param[in] v Scalar linear velocity value in m/s.
         */
        void setLinearVel(double v);

        /**
         * Update the rotation of the robot. Takes in the rotation matrix only. 
         *
         * \param[in] R Updated Rotation matrix.
         */
        void setTheta(const Eigen::Matrix3f& R);

        /**
         * Returns the current pose of the robot.
         *
         * \return 4x4 matrix denoting the current trajectory matrix.
         */
        Eigen::Matrix4f getPose();

        /**
         * Converts coordinates from robot to world frame using current 
         * robot pose. Converts one point at a time.
         *
         * \param[in] p 3x1 vector in robot frame.
         * \return Returns a 3x1 vector converted to world frame.
         */
        // Eigen::Vector3f bodyToWorld(const Eigen::Vector3f& p);

        /**
         * Converts coordinates from robot to world frame using current 
         * robot pose. Converts multiple points at once.
         *
         * \param[in] p 3xX Matrix in robot frame. X is dynamic.
         * \return Returns a 3xX matrix converted to world frame. X is same as input.
         */
        Eigen::Matrix<float, 3, Eigen::Dynamic> bodyToWorld(const Eigen::Matrix<float, 3, Eigen::Dynamic>& p);

        /**
         * Converts coordinates from world to robot frame using current
         * robot pose. One point at a time.
         *
         * \param[in] p 3x1 vector in world frame.
         * \return Returns a 3x1 vector converted to robot frame.
         */
        Eigen::Vector3f worldToBody(const Eigen::Vector3f& p);

        /**
         * Converts coordinates from world to robot frame using current
         * robot pose. Multiple points at once.
         *
         * \param[in] p 3xX Matrix in world frame. X is dynamic.
         * \return Returns a 3xX matrix converted to robot frame. X is same as input.
         */
        Eigen::Matrix<float, 3, Eigen::Dynamic> worldToBody(const Eigen::Matrix<float, 3, Eigen::Dynamic>& p);

        /**
         * Function for the motion model. Can be implemented here in the future.
         */
        void updateMotion();
        
        /**
         * Homogenize
         *
         * \param[in] v 3xX Matrix. X is dynamic. Each column is a point
         * \return Returns a 4xX Homogenized matrix. X is same as input.
         */
        Eigen::Matrix<float, 4, Eigen::Dynamic> Homogenize(const Eigen::Matrix<float, 3, Eigen::Dynamic>& v);

        /**
         * Dehomogenize
         *
         * \param[in] v 4xX Matrix. X is dynamic. Each column is a point.
         * \return Returns a 3xX Dehomogenized matrix. X is same as input.
         */
        Eigen::Matrix<float, 3, Eigen::Dynamic> Dehomogenize(const Eigen::Matrix<float, 4, Eigen::Dynamic>& v);

};