#include "gaussian_map/slam.h"

int main(int argc, char** argv){
  Eigen::Vector3f vol_res{1024,1024,1024}; /**<No of voxels per edge in volume*/
  Eigen::Vector3i vol_dim{300,300,300};
  std::string datafile = "intel.gfs.log";
  float pseudo_grid_res = 0.2, trunc_band=1.5;
  ros::init(argc, argv, "map_publisher");
  ros::NodeHandle nh;
  slam maslam(vol_res,vol_dim,datafile,pseudo_grid_res,trunc_band, nh);
  for (unsigned int i=1000;i<2000;i++){
    maslam.mapNext();
  }
  return 0;
}