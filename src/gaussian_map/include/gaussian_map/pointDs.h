#include <Eigen/Core>

/**
 * @brief typeAlloc - 0-> Node 1-> Voxelblock
 * 
 * @tparam HashType 
 */
template<typename HashType>
struct pointVals {
    HashType hash;
    Eigen::Vector3f pt;
    float sdf;
    bool typeAlloc;
};