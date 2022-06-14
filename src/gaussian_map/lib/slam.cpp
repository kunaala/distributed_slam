#include<gaussian_map/slam.h>


slam::slam(const Eigen::Vector3f vol_res, const Eigen::Vector3i vol_dim,std::string datafile, float ps_grid_res, float mu):
            agentT_(Eigen::Matrix4f::Identity()), dataseq_(datafile,0), angle_vec_(180){
    volume_dimension_ = vol_dim;
    volume_resolution_ = vol_res;
    dataFile_ = datafile;
    ps_grid_res_ = ps_grid_res;
    mu_ = mu;
    

    discrete_vol_ptr_ = std::make_shared<se::Octree<FieldType> >();
    discrete_vol_ptr_->init(volume_resolution_.x(), volume_dimension_.x());
    volume_ = Volume<FieldType>(volume_resolution_.x(), volume_dimension_.x(), discrete_vol_ptr_.get());

    for(int i=0;i<180;i++) {
        angle_vec_[i]=thetai_ + i*ang_res_;
    }
    
}

Eigen::Matrix<float, 9, 3> slam::getPseudoPts(const Eigen::Vector3f p, float grid_side) {
    Eigen::Matrix<float, 9, 3> pts;
    pts<<p(0)-grid_side, p(1)-grid_side, p(2),
        p(0), p(1)-grid_side, p(2),
        p(0)+grid_side, p(1)-grid_side, p(2),
        p(0)-grid_side, p(1), p(2),
        p(0), p(1), p(2),
        p(0)+grid_side, p(1), p(2),
        p(0)-grid_side, p(1)+grid_side, p(2),
        p(0), p(1)+grid_side, p(2),
        p(0)+grid_side, p(1)+grid_side, p(2);
    return pts;
}



float slam::getSignedDist(const Eigen::Vector3f p1, const Eigen::Vector3f p2, const Eigen::Vector3f q) {
    double dist = sqrt((p2(0)-p1(0))*(p2(0)-p1(0)) + (p1(1)-p2(1))*(p1(1)-p2(1)));
    double val = abs((p2(0)-p1(0))*(p1(1)-q(1)) - (p1(0)-q(0))*(p2(1)-p1(1)));
    return val/dist;
}



double slam::euclidDist(const Eigen::Vector3f p1, const Eigen::Vector3f p2) {
    return (p1-p2).squaredNorm();
}

bool slam::mapNext() {
    std::pair<int,Eigen::VectorXf> dataPoint = dataseq_.getNextPoint();
    if(dataPoint.first==0) { /**<ODOM datapoint*/
        // Updating robot pose
        Eigen::VectorXf vals = dataPoint.second;
        Eigen::Matrix3f Rx,Ry,Rz;

        Rx<<1,0,0,
            0,cos(vals[2]),-sin(vals[2]),
            0,sin(vals[2]),cos(vals[2]);
        Ry<<cos(vals[2]),0,sin(vals[2]),
            0,1,0,
            -sin(vals[2]),0,cos(vals[2]);
        Rz = Eigen::Matrix3f::Identity();
        Eigen::Vector3f tr(vals[0],vals[1],0.0);
        
        Eigen::Matrix4f newPose = Eigen::Matrix4f::Identity();
        newPose.block(0,0,3,3) = Rz*Ry*Rx;
        newPose.block(0,3,3,1) = tr;
        agentT_.setPose(newPose);

    }

    else if(dataPoint.first==1) { //Lidar scan datapoint
        // Update map
        float voxelsize =  volume_._dim/volume_._size;
        // unsigned int num_vox_per_pt = volume_._dim/((se::VoxelBlock<FieldType>::side)*voxelsize);
        unsigned int num_vox_per_pt = 1;
        size_t total = num_vox_per_pt * 180 * 9;
        allocation_list_.reserve(total);
        ordered_alloc_list_.reserve(total);
        keycount_per_block_.reserve(total);
        prev_alloc_list_.reserve(total);
        prev_ordered_alloc_list_.reserve(total);
        prev_keycount_per_block_.reserve(total);


        std::vector<float> float_sdf;
        Eigen::VectorXf vals = dataPoint.second;
        Eigen::Matrix<float,3,Eigen::Dynamic> pts_body;
        Eigen::Vector3f curr_pt;
        curr_pt<<cos(angle_vec_(0))*vals(0), sin(angle_vec_(0))*vals(0), 0.0;
        Eigen::Matrix<float,180*9,3> mapPts_robotFrame;
        for(int i=1;i<180;i++) {
            Eigen::Vector3f pt;
            pt<<cos(angle_vec_(i))*vals(i), sin(angle_vec_(i))*vals(i),0.0;
            Eigen::Matrix<float,9,3>  ps_pts = getPseudoPts(curr_pt,ps_grid_res_);
            mapPts_robotFrame.block(i,0,9,3) = ps_pts;
            for(int j=0;j<9;j++) {
                float d = getSignedDist(curr_pt, pt, ps_pts.row(j));
                float_sdf.push_back(d);
            }
            curr_pt = pt;
        }
        // std::clamp(float_sdf,mu_,-mu_);
        Eigen::Matrix<float,180*9,3> mapPts_worldFrame = agentT_.bodyToWorld(mapPts_robotFrame.transpose()).transpose();
        int prev_alloc_size = -1, prev_num_blockpts = -1;
        unsigned int allocated = buildAllocationList(allocation_list_.data(), allocation_list_.capacity(), prev_alloc_list_.data(),
                                    float_sdf.data(), mapPts_worldFrame, *volume_._map_index, 180*9, prev_alloc_size,
                                    prev_num_blockpts, volume_._size, voxelsize, 2*mu_);
        std::cout<<"prev num blockpts"<< prev_num_blockpts<<":"<<prev_alloc_size<<'\n';
        #if defined(_OPENMP) && !defined(__clang__)
            __gnu_parallel::sort(allocation_list_.data(),allocation_list_.data() + allocated);
            __gnu_parallel::sort(prev_alloc_list_.data(),prev_alloc_list_.data() + prev_alloc_size);
        #else
             std::sort(prev_alloc_list_.data(),prev_alloc_list_.data() + prev_alloc_size, 
                                            [](const auto& i, const auto& j) { 
                                                if(i.typeAlloc==j.typeAlloc) return i.hash < j.hash;
                                                else return (i.typeAlloc > j.typeAlloc); });
            std::sort(allocation_list_.data(),allocation_list_.data() + allocated, 
                                            [](const auto& i, const auto& j) { 
                                                if(i.typeAlloc==j.typeAlloc) return i.hash < j.hash;
                                                else return (i.typeAlloc > j.typeAlloc); });

           
        #endif

        int num_elem = se::algorithms::filter_ancestors(allocation_list_.data(), allocated, log2(volume_._size),
                                                ordered_alloc_list_.data() , keycount_per_block_.data());
        int prev_num_elem = se::algorithms::filter_ancestors(prev_alloc_list_.data(), prev_alloc_size, log2(volume_._size),
                                                prev_ordered_alloc_list_.data() , prev_keycount_per_block_.data());

        std::cout<<"num_pts"<<allocated<<"\n";
        std::cout<<"num_blocks+nodes"<<num_elem<<"\n";
        std::cout<<"Prev_num_pts"<<prev_alloc_size<<"\n";
        std::cout<<"Prev_num_pts - blocks only"<<prev_num_blockpts<<"\n";
        std::cout<<"Prev_num_blocks+nodes"<<prev_num_elem<<"\n";
        int prev_num_blocks = 0;
        for(unsigned int i=0;i<prev_num_elem;i++) {
            if(prev_ordered_alloc_list_[i].typeAlloc==0) {
                prev_num_blocks++;
            }
            else break;
        }
        std::cout<<"Prev_num_blocks only"<<prev_num_blocks<<"\n";
        std::vector<se::key_t> keys;

        std::cout<<"Num new points after compaction = Num of new blocks+nodes to be allocated :"<<num_elem<<'\n';
        for(unsigned int i =0; i < num_elem;i++){
            keys.push_back(allocation_list_[i].hash);
        }

        volume_._map_index->allocate(keys.data(), num_elem);
        
        struct sdf_update funct(mu_, maxWeight_);
        se::functor::projective_map(*volume_._map_index, funct, ordered_alloc_list_.data(), prev_ordered_alloc_list_.data(),
                                    keycount_per_block_.data(), prev_keycount_per_block_.data(),
                                    num_elem, prev_num_blocks, prev_num_elem-prev_num_blocks);
    }
    return true;
}