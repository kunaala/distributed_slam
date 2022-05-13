#include<multiagent_slam/denseSLAM.h>
#include "kfusion/alloc_impl.hpp"


denseSLAM::denseSLAM(const Eigen::Vector2i& inputSize,
                    const Eigen::Vector3i& volumeResolution,
                    const Eigen::Vector3f& volumeDimensions,
                    const Eigen::Vector3f& initPose,
                    std::vector<int> & pyramid,
                    const Configuration& config):
      denseSLAM(inputSize, volumeResolution, volumeDimensions,
          se::math::toMatrix4f(initPose), pyramid, config) { }

denseSLAM::denseSLAM(const Eigen::Vector2i& inputSize,
                                 const Eigen::Vector3i& volumeResolution,
                                 const Eigen::Vector3f& volumeDimensions,
                                 const Eigen::Matrix4f& initPose,
                                 std::vector<int> & pyramid,
                                 const Configuration& config) :
    computation_size_(inputSize) {

    this->init_pose_ = initPose.block<3,1>(0,3);
    this->volume_dimension_ = volumeDimensions;
    this->volume_resolution_ = volumeResolution;
    this->mu_ = config.mu;
    pose_ = initPose;

    this->iterations_.clear();
    for (std::vector<int>::iterator it = pyramid.begin();
        it != pyramid.end(); it++) {
      this->iterations_.push_back(*it);
    }

    viewPose_ = &pose_;

    // for (unsigned int i = 0; i < iterations_.size(); ++i) {
    //   int downsample = 1 << i;
    //   scaled_depth_.push_back(se::Image<float>(computation_size_.x() / downsample,
    //         computation_size_.y() / downsample));

    //   input_vertex_.push_back(se::Image<Eigen::Vector3f>(computation_size_.x() / downsample,
    //         computation_size_.y() / downsample));

    //   input_normal_.push_back(se::Image<Eigen::Vector3f>(computation_size_.x() / downsample,
    //         computation_size_.y() / downsample));
    // }


    discrete_vol_ptr_ = std::make_shared<se::Octree<FieldType> >();
    discrete_vol_ptr_->init(volume_resolution_.x(), volume_dimension_.x());
    volume_ = Volume<FieldType>(volume_resolution_.x(), volume_dimension_.x(), discrete_vol_ptr_.get());
}

bool denseSLAM::integration(const Eigen::Vector4f& k, unsigned int integration_rate,
    float mu, unsigned int frame) {
    if (((frame % integration_rate) == 0) || (frame <= 3)) {

        float voxelsize =  volume_._dim/volume_._size;
        int num_vox_per_pix = volume_._dim/((se::VoxelBlock<FieldType>::side)*voxelsize);
        size_t total = num_vox_per_pix * computation_size_.x() *
        computation_size_.y();
        allocation_list_.reserve(total);

        unsigned int allocated = 0;
        if(std::is_same<FieldType, SDF>::value) {
        allocated  = buildAllocationList(allocation_list_.data(),
            allocation_list_.capacity(),
            *volume_._map_index, pose_, float_depth_.data(),
            computation_size_, volume_._size, voxelsize, 2*mu);
        }
        // else if(std::is_same<FieldType, OFusion>::value) {
        //     allocated = buildOctantList(allocation_list_.data(), allocation_list_.capacity(),
        //     *volume_._map_index,
        //     pose_, getCameraMatrix(k), float_depth_.data(), computation_size_, voxelsize,
        //     compute_stepsize, step_to_depth, 6*mu);
        // }

        volume_._map_index->allocate(allocation_list_.data(), allocated);

        if(std::is_same<FieldType, SDF>::value) {
            struct sdf_update funct(float_depth_.data(),
                Eigen::Vector2i(computation_size_.x(), computation_size_.y()), mu, 100);
            se::functor::projective_map(*volume_._map_index,
                Sophus::SE3f(pose_).inverse(),
                getCameraMatrix(k),
                Eigen::Vector2i(computation_size_.x(), computation_size_.y()),
                funct);
        } 
        // else if(std::is_same<FieldType, OFusion>::value) {

        // float timestamp = (1.f/30.f)*frame;
        // struct bfusion_update funct(float_depth_.data(),
        //     Eigen::Vector2i(computation_size_.x(), computation_size_.y()), 
        //     mu, timestamp, voxelsize);

        // se::functor::projective_map(*volume_._map_index,
        //     Sophus::SE3f(pose_).inverse(),
        //     getCameraMatrix(k),
        //     Eigen::Vector2i(computation_size_.x(), computation_size_.y()),
        //     funct);
        // }
    } else {
        return false;
    }
    return true;
}


// When to apply gp regression.
// What to do the new points overlap with existing points which gp is computed.
// Only update points in the map or update all points along the ray for each point in the band.
// It is kind of difficult to do for 2d data. Can directly translate to 3d
