/*
	Copyright 2016 Emanuele Vespa, Imperial College London 
	Redistribution and use in source and binary forms, with or without
	modification, are permitted provided that the following conditions are met:

	1. Redistributions of source code must retain the above copyright notice, this
	list of conditions and the following disclaimer.

	2. Redistributions in binary form must reproduce the above copyright notice,
	this list of conditions and the following disclaimer in the documentation
	and/or other materials provided with the distribution.

	3. Neither the name of the copyright holder nor the names of its contributors
	may be used to endorse or promote products derived from this software without
	specific prior written permission.

	THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
	ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
	WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
	DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
	FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
	DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
	SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
	CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
	OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
	OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. 

*/

#ifndef PROJECTIVE_FUNCTOR_HPP
#define PROJECTIVE_FUNCTOR_HPP
#include <functional>
#include <vector>

#include <sophus/se3.hpp>
#include "../utils/math_utils.h"
#include "../algorithms/filter.hpp"
#include "../node.hpp"
#include "../functors/data_handler.hpp"

namespace se {
namespace functor {
  	template <typename FieldType, template <typename FieldT> class MapT, typename UpdateF, typename PointStr>
  	class projective_functor {

		public:
			EIGEN_MAKE_ALIGNED_OPERATOR_NEW
			/**
			 * @brief 	Construct a new projective functor object
			 * 
			 * @param[out] map  					Octree structure/ Volume template
			 * @param[in] f  						Projective functor
			 * @param[in] allocList 				Newly allocated voxel blocks
			 * @param[in] old_allocList 			previously allocated voxel blocks
			 * @param[in] num_keys_per_block 		Number of keys/block in Newly allocated voxel blocks
			 * @param[in] old_num_keys_per_block 	Number of keys/block in previously allocated voxel blocks
			 * @param[in] num_elem 					Number of new blocks+nodes to be allocated
			 * @param[in] num_prealloc_blocks 		Number of previously allocated blocks	
			 * @param[in] num_prealloc_nodes 		Number of previously allocated nodes
			 */
			projective_functor(MapT<FieldType>& map, UpdateF f,
							PointStr* allocList, PointStr* old_allocList, int* num_keys_per_block,
							int* old_num_keys_per_block, int num_elem, int num_prealloc_blocks, int num_prealloc_nodes) : 
			_map(map), _function(f), _allocList(allocList), _old_allocList(old_allocList),
			_keycount_per_block(num_keys_per_block), _old_keycount_per_block(old_num_keys_per_block),
			num_elem_(num_elem), num_prealloc_blocks_(num_prealloc_blocks), num_prealloc_nodes_(num_prealloc_nodes) {}

		void build_active_list() {
			using namespace std::placeholders;
			/* Retrieve the active list */ 
			const se::MemoryPool<se::VoxelBlock<FieldType> >& block_array = _map.getBlockBuffer();

			/* Predicates definition */
			const float voxel_size = _map.dim()/_map.size();
			// auto in_frustum_predicate = 
			//   std::bind(algorithms::in_frustum<se::VoxelBlock<FieldType>>, _1, 
			//       voxel_size, _K*_Tcw.matrix(), _frame_size); 
			auto in_frustum_predicate = [](const se::VoxelBlock<FieldType>* b) {
				return false;
			};
			auto is_active_predicate = [](const se::VoxelBlock<FieldType>* b) {
				return b->active();

			};
			/**
			 * removes blocks not part of current active list
			 */
			algorithms::filter(_active_list, block_array, is_active_predicate,
				in_frustum_predicate);
		}

		void update_block(se::VoxelBlock<FieldType> * block, int kind, const float voxel_size, 
							const int start, const int end) {
			bool is_visible = false;
			if(kind==0) {
				for(unsigned int idx = start;idx<end;idx++) {
					VoxelBlockHandler<FieldType> handler = {block, _old_allocList[idx].pt};
					_function(handler, _old_allocList[idx].sdf);
				}
			}
			else {
				for(unsigned int idx = start;idx<end;idx++) {
					// if(_allocList[idx].typeAlloc!=0) std::cout<<"Not block"<<'\n';
					// else std::cout<<"Yes block\n";
					VoxelBlockHandler<FieldType> handler = {block, _allocList[idx].pt};
					// VoxelBlockHandler<FieldType> handler = {block, unpack_morton(block->code_)};
					_function(handler, _allocList[idx].sdf);
				}
			}

			is_visible = true;

			// const Eigen::Vector3i blockCoord = block->coordinates();
			// const Eigen::Vector3f delta = Eigen::Vector3f(voxel_size, 0, 0);
			// const Eigen::Vector3f cameraDelta = _K.topLeftCorner<3,3>() * delta;
			// unsigned int y, z, blockSide; C
			// 		for (unsigned int x = 0; x < blockSide; ++x){
			// 			pix(0) = x + blockCoord(0); 
			// 			const Eigen::Vector3f camera_voxel = camerastart + (x*cameraDelta);
			// 			const Eigen::Vector3f pos = start + (x*delta);
			// 			if (pos(2) < 0.0001f) continue;

			// 			const float inverse_depth = 1.f / camera_voxel(2);
			// 			const Eigen::Vector2f pixel = Eigen::Vector2f(
			// 				camera_voxel(0) * inverse_depth + 0.5f,
			// 				camera_voxel(1) * inverse_depth + 0.5f);
			// 			if (pixel(0) < 0.5f || pixel(0) > _frame_size(0) - 1.5f || 
			// 				pixel(1) < 0.5f || pixel(1) > _frame_size(1) - 1.5f) continue;
			// 			is_visible = true;

			// 			VoxelBlockHandler<FieldType> handler = {block, pix};
			// 			_function(handler, pix, pos, pixel);
			// 		}
			// 	}
			block->active(false);

		}

		void update_node(se::Node<FieldType> * node, int kind, const float voxel_size, const int idx) {
			const Eigen::Vector3i voxel = Eigen::Vector3i(unpack_morton(node->code_));
			int i = (int)(node->code_)>>9;
			std::cout<<node->code_<<"Node coors------\n";
			NodeHandler<FieldType> handler = {node, i};
			if(kind==0) _function(handler, _old_allocList[idx].sdf);
			else _function(handler, _allocList[idx].sdf);
			
			// const Eigen::Vector3i voxel = Eigen::Vector3i(unpack_morton(node->code_));
			// const Eigen::Vector3f delta = _Tcw.rotationMatrix() * Eigen::Vector3f::Constant(0.5f * voxel_size * node->side_);
			// const Eigen::Vector3f delta_c = _K.topLeftCorner<3,3>() * delta;
			// Eigen::Vector3f base_cam = _Tcw * (voxel_size * voxel.cast<float> ());
			// Eigen::Vector3f basepix_hom = _K.topLeftCorner<3,3>() * base_cam;

			// #pragma omp simd
			// for(int i = 0; i < 8; ++i) {
			// 	const Eigen::Vector3i dir =  Eigen::Vector3i((i & 1) > 0, (i & 2) > 0, (i & 4) > 0);
			// 	const Eigen::Vector3f vox_cam = base_cam + dir.cast<float>().cwiseProduct(delta); 
			// 	const Eigen::Vector3f pix_hom = basepix_hom + dir.cast<float>().cwiseProduct(delta_c); 

			// 	if (vox_cam(2) < 0.0001f) continue;
			// 	const float inverse_depth = 1.f / pix_hom(2);
			// 	const Eigen::Vector2f pixel = Eigen::Vector2f(
			// 		pix_hom(0) * inverse_depth + 0.5f,
			// 		pix_hom(1) * inverse_depth + 0.5f);
			// 	if (pixel(0) < 0.5f || pixel(0) > _frame_size(0) - 1.5f || 
			// 		pixel(1) < 0.5f || pixel(1) > _frame_size(1) - 1.5f) continue;

				// NodeHandler<FieldType> handler = {node, i};
			// 	_function(handler, voxel + dir, vox_cam, pixel);
			// }

		}

		void apply() {
			build_active_list();
			const float voxel_size = _map.dim() / _map.size();
			size_t list_size = _active_list.size();
			std::cout<<"Active List size Blocks:- -- - - -- - - -- - ----- "<<list_size<<"\n";

			unsigned int prev_new = 0;
			unsigned int i = 0;
			/**< sort previously allocated blocks */
			std::sort(_active_list.data(), _active_list.data() + num_prealloc_blocks_, [](const auto& aa, const auto& bb){return aa->code_<bb->code_;});
			std::cout<<"Active list hashes print -----\n";
			for(int qq=0; qq < list_size; qq++) std::cout<<_active_list[qq]->code_<<"\t";
			std::cout<<"\n";
			#pragma omp parallel for
			for(; i < num_prealloc_blocks_; i++) {
				std::cout<<_active_list[i]->coordinates()<<"block -------------\n";
				std::cout<<"keycount -------------:"<<_old_keycount_per_block[i]<<"\n";
				std::cout<<_active_list[i]->code_<<"Active block code.....\n";
				std::cout<<_old_allocList[prev_new].hash<<"List code---\n";
				update_block(_active_list[i], 0, voxel_size, prev_new, prev_new+_old_keycount_per_block[i]);
				prev_new += _old_keycount_per_block[i];
			}
			unsigned int prev = 0;

			#pragma omp parallel for
			for(; i < list_size; ++i){
				// std::cout<<"noe"<<_active_list[i]<<" "<<_keycount_per_block[i]<<"\n";
				std::cout<<_active_list[i]->coordinates()<<"block -------------\n";
				std::cout<<_keycount_per_block[i-num_prealloc_blocks_]<<"keycount -------------\n";
				update_block(_active_list[i], 1, voxel_size, prev, prev+_keycount_per_block[i-num_prealloc_blocks_]);
				prev += _keycount_per_block[i-num_prealloc_blocks_];
			}
			_active_list.clear();
			std::cout<<"cleared active list"<<'\n';

			auto& nodes_list = _map.getNodesBuffer();
			list_size = nodes_list.size();
			std::cout<<"Active List size Nodes:- -- - - -- - - -- - ----- "<<list_size<<"\n";
			// i = 0;
			// std::cout<<" num nodes"<<list_size<<'\n';
			// #pragma omp parallel for
			// for(; i < num_prealloc_nodes_; ++i){
			// 	update_node(nodes_list[i], 0, voxel_size, prev_new);
			// 	prev_new++;
			// }

			// #pragma omp parallel for
			// for(; i < list_size; ++i){
			// 	update_node(nodes_list[i], 1, voxel_size,prev);
			// 	prev++;
			// }
		}

		private:
			MapT<FieldType>& _map; 
			UpdateF _function; 
			// Sophus::SE3f _Tcw;
			// Eigen::Matrix4f _K;
			// Eigen::Vector2i _frame_size;
			std::vector<se::VoxelBlock<FieldType>*> _active_list;
			PointStr* _allocList;
			PointStr* _old_allocList;
			int* _keycount_per_block;
			int* _old_keycount_per_block;
			int num_elem_;
			int num_prealloc_blocks_;
			int num_prealloc_nodes_;

  	};

	template <typename FieldType, template <typename FieldT> class MapT, 
				typename UpdateF, typename PointStr>
	void projective_map(MapT<FieldType>& map, UpdateF funct,
					PointStr* allocList, PointStr* old_allocList, int* num_keys_per_block,
					int* old_num_keys_per_block, int num_elem, int num_prealloc_blocks, int num_prealloc_nodes) {
		projective_functor<FieldType, MapT, UpdateF,PointStr> 
		it(map, funct, allocList, old_allocList, num_keys_per_block, old_num_keys_per_block,
			num_elem, num_prealloc_blocks, num_prealloc_nodes);
		it.apply();
		std::cout<<"---------------------********************"<<"\n";
	}
}
}
#endif
