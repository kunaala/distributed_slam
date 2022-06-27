#ifndef _KERNELS_
#define _KERNELS_

#include <iostream>
#include <cstdlib>
#include <vector>
#include <map>
#include <memory>
#include <algorithm>
#include <Eigen/Dense>
#include "config.h"
#include <gaussian_map/octree.hpp>
#include <gaussian_map/robot.h>
#include <gaussian_map/dataloader.h>
// #include <gaussian_map/pointDs.h>
#include <gaussian_map/kfusion/alloc_impl.hpp>
#include <gaussian_map/kfusion/mapping_impl.hpp>
#include <gaussian_map/volume_traits.hpp>
#include<gaussian_map/volume_template.hpp>
#include "gaussian_map/algorithms/unique.hpp"
#include "gaussian_map/functors/projective_functor.hpp"





typedef SE_FIELD_TYPE FieldType;
template <typename T>
using Volume = VolumeTemplate<T, se::Octree>;

class slam {
    protected:
        Eigen::Vector3f volume_resolution_;
        Eigen::Vector3i volume_dimension_;
        std::vector<struct pointVals<se::key_t>> allocation_list_;
        std::vector<struct pointVals<se::key_t>> prev_alloc_list_;
        std::vector<struct pointVals<se::key_t>> unfiltered_alloc_list_;
        std::vector<struct pointVals<se::key_t>> prev_unfiltered_alloc_list_;
        std::vector<int> keycount_per_block_;
        std::vector<int> prev_keycount_per_block_;
        std::vector<float> float_depth_;
        std::shared_ptr<se::Octree<FieldType> > discrete_vol_ptr_;
        Volume<FieldType> volume_;
        std::string dataFile_;
        robot agentT_;
        dataloader dataseq_;
        float thetai_ = -1.570796;
        float ang_res_ = 2*0.008727;
        float ps_grid_res_ = 0.1;
        float mu_ = 3.f;
        float maxWeight_ = 100.f;
        Eigen::VectorXf angle_vec_;

    public:
        /**
         * Constructor. Initialises volume.
         * 
         * \param[in] vol_res The x, y and z resolution of the
         * reconstructed volume in voxels.
         * \param[in] vol_dim The x, y and z dimensions of the
         * \param[in] ps_grid_res length of the side of the 3x3 pseudo point grid.
         * reconstructed volume in meters.
         */
        slam(const Eigen::Vector3f vol_res, const Eigen::Vector3i vol_dim, std::string datafile, float ps_grid_res, float mu);

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
        Eigen::Matrix<float, 9, 3> getPseudoPts(const Eigen::Vector3f p, float grid_side);

        /**
         * Get the TSDF value of the target point using two consequtive laser hitpoints.
         *
         * \param[in] p1 3x1 vector denoting point one.
         * \param[in] p2 3x1 vector denoting point two.
         * \param[in] q 3x1 vector denoting the target point.
         * \return Returns the TSDF value at target point.
         */
        float getSignedDist(const Eigen::Vector3f p1, const Eigen::Vector3f p2, const Eigen::Vector3f q);

        /**
         * Computes the euclidean distance between two points.
         *
         * \param[in] p1 3x1 vector point 1
         * \param[in] p2 3x1 vector point 2.
         * \return Returns the euclidean distance between two points.
         */
        double euclidDist(const Eigen::Vector3f p1, const Eigen::Vector3f p2);

        /**
         * Maps the next point from the data sequence.
         *
         * \return Returns true if mapping succesful. False if not or data complete.
         */
        bool mapNext();

        unsigned int update_count_ =0;

};
#endif