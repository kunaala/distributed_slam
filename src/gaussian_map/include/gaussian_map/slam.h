#ifndef _KERNELS_
#define _KERNELS_

#include <iostream>
#include <cstdlib>
#include <vector>
#include <unordered_map>
#include <memory>
#include <algorithm>
#include <chrono>
#include <thread>
#include <valarray>
#include <string>
#include <Eigen/Core>
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
#include "gaussian_map/SparseGp.hpp"
#include "gaussian_map/utils/gp_utils.h"
#include "gaussian_map/MapPub.hpp"
#include "ros/ros.h"




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
        const unsigned int num_pseudo_pts_=9;
        float voxel_size_;
        int prev_alloc_size_;
        unsigned int allocated_;
        Eigen::MatrixX3f X_train_;
        std::vector<Eigen::Vector3i> block_coords_;
        std::unordered_map<int,std::vector<Eigen::Vector3i>> block_hash_mapped_pseudo_pts;
        std::unordered_map<int,Eigen::Vector3i> block_hash_mapped_block_coords;
        ros::NodeHandle nh_;

        /**<functor to sort blocks in allocation list*/
         struct less_than_key {
            inline bool operator() (const pointVals<se::key_t>& struct1, const pointVals<se::key_t>& struct2) {
                if(struct1.typeAlloc!=struct2.typeAlloc) return struct1.typeAlloc<struct2.typeAlloc;
                return struct1.hash < struct2.hash;
            }
        };
        Eigen::MatrixXf retrieve_sdf(se::Octree<FieldType> *map_index, Eigen::MatrixX3f voxelPos);
        std::vector<int8_t> gen_grid(std::vector<Eigen::MatrixXf> &D);
        
        void group_training_pts(se::Octree<FieldType> *map_index);
        Eigen::MatrixXf gen_test_pts(Eigen::Vector3i block_coord);
        void predict(se::Octree<FieldType> *map_index);





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
        slam(const Eigen::Vector3f vol_res, const Eigen::Vector3i vol_dim,std::string datafile, float ps_grid_res, float mu, ros::NodeHandle &nh);
        /**
         * Maps the next point from the data sequence.
         *
         * \return Returns true if mapping succesful. False if not or data complete.
         */
        bool mapNext();
        void visualize(const std::vector<Eigen::MatrixXf> &D);

        unsigned int update_count_ = 1;
        float map_res_ = 0.1f;
        unsigned int blockSide_;
        std::pair<unsigned int,unsigned int> map_size_ = std::make_pair(300,300); 
        unsigned int predict_cycle_ = 3;
        unsigned int filter_training_ = 2;
        std::string fname_ = "map_ros.txt";
        std::vector<int8_t> ros_map_;
        Eigen::VectorXf curr_odom_;
        Eigen::MatrixX2f odom_vals;



};
#endif