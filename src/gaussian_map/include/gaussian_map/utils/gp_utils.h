#include<Eigen/Dense>
#include<vector>
/**
 * @brief Compute angle increments for Lidar scan
 * @param scan_angle maximum scan angle of the Lidar
 */
Eigen::VectorXf angle_increments(unsigned int span, float start_angle, float angle_res){
    Eigen::VectorXf angle_vec(span);
    for(int i=0;i<180;i++) {
        angle_vec(i)= start_angle + i*angle_res;
    }
    return angle_vec;
}
/**
 * Get the TSDF value of the target point using two consequtive laser hitpoints.
 *
 * \param[in] p1 3x1 vector denoting point one.
 * \param[in] p2 3x1 vector denoting point two.
 * \param[in] q 3x1 vector denoting the target point.
 * \return Returns the TSDF value at target point.
 */
float getSignedDist(const Eigen::Vector3f p1, const Eigen::Vector3f p2, const Eigen::Vector3f q) {
    double dist = sqrt((p2(0)-p1(0))*(p2(0)-p1(0)) + (p1(1)-p2(1))*(p1(1)-p2(1)));
    double val = abs((p2(0)-p1(0))*(p1(1)-q(1)) - (p1(0)-q(0))*(p2(1)-p1(1)));
    return val/dist;
}

/**
 * Computes the euclidean distance between two points.
 *
 * \param[in] p1 3x1 vector point 1
 * \param[in] p2 3x1 vector point 2.
 * \return Returns the euclidean distance between two points.
 */
double euclidDist(const Eigen::Vector3f p1, const Eigen::Vector3f p2) {
    return (p1-p2).squaredNorm();
}

/**
 * Computes the location the pseudo points by placing a 3x3 grid on the laser hitpoint.
 *
 * \param[in] p 3x1 vector denoting the laser hitpoint.
 * \param[in] grid_side length of the side of the pseudo point grid.
 * \param[in] datafile string denoting the file location of the dataset.
 * \param[in] mu the band width for the TSDF.
 * \return Returns 9x3 matrix where each row is a pseudo point At
 * the center of the 3x3 pseudo point grid.
 */

Eigen::Matrix<float, 9, 3> getPseudoPts(const Eigen::Vector3f p, float grid_side) {
    Eigen::Matrix<float, 9, 3> pts;
    pts<<p(0)-grid_side, p(1)-grid_side, p(2),
        p(0), p(1)-grid_side, p(2),
        p(0)+grid_side, p(1)-grid_side, p(2),
        p(0)-grid_side, p(1), p(2),
        p(0), p(1), p(2),
        p(0)+grid_side, p(1), p(2),
        p(0)-grid_side, p(1)+grid_side, p(2),
        p(0), p(1)+grid_side, p(2),
        p(0)+grid_side, p(1)+grid_side, p(2);
    return pts;
}

std::pair<Eigen::MatrixXf,std::vector<float>> gen_pseudo_pts(Eigen::VectorXf range_vals, 
                                                float ps_grid_side, unsigned int num_pseudo_pts
                                                ){
    /**
     * @brief 1. Generates num_pseudo_pts with reference to laser hit point
     * @brief 2. calculates their sdf values using getSignedDist function
     * @brief 3. Converts all pseudo points(includes hit points) into World Frame
     */
    std::vector<float> sdf_vals;
    Eigen::VectorXf vals = range_vals;
    // Eigen::Matrix<float,3,Eigen::Dynamic> pts_body; /**<3D vector*/
    Eigen::Vector3f curr_pt;
    unsigned int span = 180;
    float start_angle = -1.570796;
    float ang_res = 2*0.008727;
    Eigen::VectorXf angle_vec = angle_increments(span, start_angle , ang_res);

    curr_pt<<   cos(angle_vec(0))*vals(0), 
                sin(angle_vec(0))*vals(0), 
                0.0;

    Eigen::MatrixXf mapPts_robotFrame(span*num_pseudo_pts,3);

    for(int i=0;i<span;i++) {
        Eigen::Vector3f pt;
        pt<<    cos(angle_vec(i))*vals(i),
                sin(angle_vec(i))*vals(i),
                0.0;
        Eigen::MatrixXf  ps_pts = getPseudoPts(curr_pt,ps_grid_side);
        mapPts_robotFrame.block(num_pseudo_pts*i,0,num_pseudo_pts,3) = ps_pts;
        for(int j=0;j<num_pseudo_pts;j++) {
            float d = getSignedDist(curr_pt, pt, ps_pts.row(j));
            sdf_vals.push_back(d);
        }
        curr_pt = pt;
    }

    // std::clamp(float_sdf,mu_,-mu_);
    //returns points in robot_frame
    return std::make_pair(mapPts_robotFrame,sdf_vals);

}

Eigen::MatrixXf gen_test_pts(std::pair<unsigned int, unsigned int> map_size, float resolution){
    /**
     * @brief generates uniformly distributed test points across the map grid
     * 
     */
    unsigned int size_x = floor(map_size.first/resolution);
    unsigned int size_y = floor(map_size.second/resolution);
    Eigen::MatrixXf test_pts(size_x * size_y,3);
	Eigen::RowVector3f pt;
	for (unsigned int i = 0; i < size_x; i++)
	{
		for (unsigned int j = 0; j < size_y; j++)
		{
			pt << i*resolution, j*resolution, 0.f;
			test_pts.row(i*size_y + j) = pt; 
		}
	}
	return test_pts;
}


void save_data(const Eigen::MatrixXf M,std::string fname){

    /*
     * Saves Eigen Matrix "M" in the file specified in "filename" in CSV format.
     */
    const static Eigen::IOFormat CSVFormat(Eigen::FullPrecision, Eigen::DontAlignCols, " ");
    std::ofstream fd(fname,std::ios::app);
	if (fd.is_open()){
        fd << M.format(CSVFormat);
        fd <<'\n';
		fd.close();
	}
}