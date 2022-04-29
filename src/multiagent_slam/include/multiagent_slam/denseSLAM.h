#ifndef _KERNELS_
#define _KERNELS_

#include <iostream>
#include <cstdlib>
#include <vector>
#include <memory>
#include <Eigen/Dense>
#include <config.h>
#include <multiagent_slam/octree.hpp>


typedef SE_FIELD_TYPE FieldType;
template <typename T>
using Volume = VolumeTemplate<T, se::Octree>;

class denseSLAM {
    private:
        Eigen::Vector2i computation_size_;
        Eigen::Matrix4f pose_;
        Eigen::Matrix4f *viewPose_;
        Eigen::Vector3f volume_dimension_;
        Eigen::Vector3i volume_resolution_;
        std::vector<int> iterations_;
        bool tracked_;
        bool integrated_;
        Eigen::Vector3f init_pose_;
        float mu_;
        bool need_render_ = false;
        Configuration config_;

        // input once
        std::vector<float> gaussian_;

        std::vector<se::key_t> allocation_list_;
        std::shared_ptr<se::Octree<FieldType> > discrete_vol_ptr_;
        Volume<FieldType> volume_;

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        /**
         * Constructor using the initial LIDAR position.
         *
         * \param[in] inputSize The size (width and height) of the input frames.
         * \param[in] volume_resolution_ The x, y and z resolution of the
         * reconstructed volume in voxels.
         * \param[in] volume_dimension_ The x, y and z dimensions of the
         * reconstructed volume in meters.
         * \param[in] initPose The x, y and z coordinates of the initial LIDAR
         * position. The LIDAR orientation is assumed to be aligned with the axes.
         * \param[in] pyramid TODO See ::Configuration.pyramid for more details.
         * \param[in] config_ The pipeline options.
         */
        denseSLAM(const Eigen::Vector2i& inputSize,
                        const Eigen::Vector3i& volume_resolution_,
                        const Eigen::Vector3f& volume_dimension_,
                        const Eigen::Vector3f& initPose,
                        std::vector<int> &     pyramid,
                        const Configuration&   config_);
        /**
         * Constructor using the initial LIDAR position.
         *
         * \param[in] inputSize The size (width and height) of the input frames.
         * \param[in] volume_resolution_ The x, y and z resolution of the
         * reconstructed volume in voxels.
         * \param[in] volume_dimension_ The x, y and z dimensions of the
         * reconstructed volume in meters.
         * \param[in] initPose The initial LIDAR pose encoded in a 4x4 matrix.
         * \param[in] pyramid TODO See ::Configuration.pyramid for more details.
         * \param[in] config_ The pipeline options.
         */
        denseSLAM(const Eigen::Vector2i& inputSize,
                        const Eigen::Vector3i& volume_resolution_,
                        const Eigen::Vector3f& volume_dimension_,
                        const Eigen::Matrix4f& initPose,
                        std::vector<int> &     pyramid,
                        const Configuration&   config_);

        
        /**
         * Integrate the 3D reconstruction resulting from the current frame to the
         * existing reconstruction. This is the third stage of the pipeline.
         *
         * \param[in] k The intrinsic camera parameters. See
         * ::Configuration.camera for details.
         * \param[in] integration_rate Integrate a 3D reconstruction every
         * integration_rate frames. Should not be less than the tracking_rate used
         * in tracking().
         * \param[in] mu TSDF truncation bound. See ::Configuration.mu for more
         * details.
         * \param[in] frame The index of the current frame (starts from 0).
         * \return true if the current 3D reconstruction was added to the octree
         * and false if it wasn't.
         */
        bool integration(const Eigen::Vector4f& k,
                        unsigned               integration_rate,
                        float                  mu,
                        unsigned               frame);

};

#endif