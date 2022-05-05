#ifndef SDF_ALLOC_H
#define SDF_ALLOC_H
#include <multiagent_slam/utils/math_utils.h> 
#include <multiagent_slam/node.hpp>
#include <multiagent_slam/utils/morton_utils.hpp>


/* 
 * \brief Given a depth map and camera matrix it computes the list of 
 * voxels intersected but not allocated by the rays around the measurement m in
 * a region comprised between m +/- band. 
 * \param allocationList output list of keys corresponding to voxel blocks to
 * be allocated
 * \param reserved allocated size of allocationList
 * \param mapPts 3D map locations of the points to be mapped
 * \param map_index indexing structure used to index voxel blocks 
 * \param num_pts number of points to be mapped
 * \param size discrete extent of the map, in number of voxels
 * \param voxelSize spacing between two consegutive voxels, in metric space
 * \param band maximum extent of the allocating region, per ray
 */

template <typename FieldType, template <typename> class OctreeT, typename HashType>
unsigned int buildAllocationList(HashType* allocationList, size_t reserved, const Eigen::Matrix<float,180*9,3> mapPts,
                            OctreeT<FieldType>& map_index, const unsigned int num_pts,
                            const unsigned int size, const float voxelSize, const float band) {
    
    const float inverseVoxelSize = 1/voxelSize;
    const unsigned block_scale = log2(size) - se::math::log2_const(se::VoxelBlock<FieldType>::side);


#ifdef _OPENMP
    std::atomic<unsigned int> voxelCount;
#else
    unsigned int voxelCount;
#endif

#pragma omp parallel for
    for(int i=0;i<num_pts;i++) {
        Eigen::Vector3i voxel;
        Eigen::Vector3f voxelPos = mapPts.row(i);
        Eigen::Vector3f voxelScaled = (voxelPos * inverseVoxelSize).array().floor();
        if( (voxelScaled.x() < size) && (voxelScaled.y() < size) && 
            (voxelScaled.z() < size) && (voxelScaled.x() >= 0) &&
            (voxelScaled.y() >= 0) &&   (voxelScaled.z() >= 0)) {
            
            voxel = voxelScaled.cast<int>();
            se::VoxelBlock<FieldType> * n = map_index.fetch(voxel.x(), voxel.y(), voxel.z());
            if(!n) {
                HashType k = map_index.hash(voxel.x(), voxel.y(), voxel.z(), block_scale);
                unsigned int idx = voxelCount++;
                if(idx < reserved) {
                    allocationList[idx] = k;
                }
                else break;
            }
            else {
                n->active(true);
            }    
        }
    }
    const unsigned int written = voxelCount;
    return written >= reserved ? reserved : written;
}
#endif