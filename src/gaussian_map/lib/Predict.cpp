#include <iostream>
#include <vector>
#include <Eigen/Dense>

#include <gaussian_map/kfusion/alloc_impl.hpp>
#include <gaussian_map/kfusion/mapping_impl.hpp>
#include <gaussian_map/volume_traits.hpp>
#include <gaussian_map/volume_template.hpp>
#include "gaussian_map/algorithms/unique.hpp"
#include "gaussian_map/functors/projective_functor.hpp"
#include <gaussian_map/octree.hpp>

template <typename FieldType>
class Predict{

    public:
        template <template FieldType>>
        Predict::Predict(Octree<FieldType>& volume): map_(volume){

        }
    private:
        const Octree<FieldType>& map_;
        std::vector<se::VoxelBlock<FieldType>*> blocklist;

        std::vector<se::key_t> Predict::find_block(std::vector<Eigen::Vector3f> coords){
            volume.getBlockList(blocklist, false);
            std::cout << "Blocklist size: " << blocklist.size() << std::endl;
            
        }

};

