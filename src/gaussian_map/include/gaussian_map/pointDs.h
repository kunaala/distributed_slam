#include <Eigen/Core>

/**
 * @brief  Structure to hold voxel keys(Hash), its 3D coordinates
 * TSDF values and the grouping to hold them Node/Voxelblock 
 * 
 */
template<typename HashType>
struct pointVals {
    HashType hash;
    Eigen::Vector3f pt;
    float sdf;
    bool typeAlloc; /**< 0-> Node 1-> Voxelblock */
};