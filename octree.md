# Octree integration into multi agent slam

## constructor -Initialises the octree attributes.
template<typename T >
void se::Octree< T >::init 	(int size,float dim ); 		
*Parameters*
    size:	number of voxels per side of the cube
    dim:	cube extension per side, in meters

## Structures and data members
  
### Octree< T >::value_type =  SDF/Ofusion -- defined in voxel_traits.hpp, populated in volume_traits.hpp
 **for KFusion Truncated Signed Distance Function voxel traits**
  
 typedef struct {
   float x;
   float y;
 } SDF;
  
 template<>
 struct voxel_traits<SDF> {
   typedef SDF value_type;
   static inline value_type empty(){ return {1.f, -1.f}; }
   static inline value_type initValue(){ return {1.f, 0.f}; }
 };
  
 
 **for Bayesian Fusion voxel traits and algorithm**
  
 typedef struct {
     float x;
     double y;
 } OFusion;
  
 template<>
 struct voxel_traits<OFusion> {
   typedef struct  {
     float x;
     double y;
   } value_type;
   static inline value_type empty(){ return {0.f, 0.f}; }
   static inline value_type initValue(){ return {0.f, 0.f}; }
 };
### Number of voxels per side in a voxel block
 *default* = 8 :  blockSide ,BLOCK_SIDE;
### maximum tree depth in bits
 max_depth = ((sizeof(key_t)*8)/3);
 where key_t is typedef uint64_t
### Tree depth at which blocks are found
 block_depth = max_depth - math::log2_const(BLOCK_SIDE);
 where log2_const(n) = n < 2 ? 0 : 1 + log2_const(n/2)
    
## Methods
### get function - Retrieves voxel value at coordinates (x,y,z) 
Octree< T >::value_type se::Octree< T >::get 	(const int x, const int  	y,const int  	z)

*Parameters*
    x	x-coordinate in interval [0, size]
    y	y-coordinate in interval [0, size]
    z	z-coordinate in interval [0, size] 
