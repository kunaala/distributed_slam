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

std::pair<Eigen::MatrixXf,std::vector<float>> slam::gen_pseudo_pts(Eigen::VectorXf range_vals){
    /**
     * @brief 1. Generates num_pseudo_pts with reference to laser hit point
     * @brief 2. calculates their sdf values using getSignedDist function
     * @brief 3. Converts all pseudo points(includes hit points) into World Frame
     */
    std::vector<float> sdf_vals;
    Eigen::VectorXf vals = range_vals;
    // Eigen::Matrix<float,3,Eigen::Dynamic> pts_body; /**<3D vector*/
    Eigen::Vector3f curr_pt;
    curr_pt<<   cos(angle_vec_(0))*vals(0), 
                sin(angle_vec_(0))*vals(0), 
                0.0;
    Eigen::MatrixXf mapPts_robotFrame(180*num_pseudo_pts_,3);

    for(int i=1;i<180;i++) {
        Eigen::Vector3f pt;
        pt<<    cos(angle_vec_(i))*vals(i),
                sin(angle_vec_(i))*vals(i),
                0.0;
        Eigen::MatrixXf  ps_pts = getPseudoPts(curr_pt,ps_grid_res_);
        mapPts_robotFrame.block(i,0,num_pseudo_pts_,3) = ps_pts;
        for(int j=0;j<num_pseudo_pts_;j++) {
            float d = getSignedDist(curr_pt, pt, ps_pts.row(j));
            sdf_vals.push_back(d);
        }
        curr_pt = pt;
    }
    // std::clamp(float_sdf,mu_,-mu_);
    Eigen::Matrix<float,180*9,3> mapPts_worldFrame = agentT_.bodyToWorld(mapPts_robotFrame.transpose()).transpose();

    return std::make_pair(mapPts_worldFrame,sdf_vals);

}

Eigen::MatrixXf slam::gen_test_pts(Eigen::MatrixX3i block_centers, const unsigned int blockSide, float voxelsize){
    Eigen::MatrixXf test_pts(block_centers.rows()*blockSide*blockSide,3);
    Eigen::VectorXf lin_v(Eigen::VectorXf::LinSpaced(blockSide+1,-(blockSide*voxelsize/2),blockSide*voxelsize/2));
    
    for(unsigned int c=0;c<block_centers.rows();c++){
        for(unsigned int i=0;i<blockSide;i++){
            for(unsigned int j=0;j<blockSide;j++)   {
                Eigen::RowVector3f pt;
                pt << lin_v[i], lin_v[j], block_centers(c,2);
                test_pts.row(c*blockSide*blockSide + i*blockSide+j) = pt;
            }

        }
    }
    return test_pts;
}
void  slam::predict(std::vector<Eigen::MatrixXf> &D, se::Octree<FieldType> *map_index, unsigned int num_elem, float voxelsize){
    std::vector<se::VoxelBlock<FieldType>*> block_list;
    const unsigned int blockSide = map_index->blockSide;
    block_list.reserve(num_elem);
    map_index->getBlockList(block_list,0);
    Eigen::MatrixXi block_centers(num_elem,3);
    for(unsigned int b=0;b<num_elem;b++) {
        // std::cout<<"block coordinates of "<<b<<"\n"<<block_list[b]->coordinates()<<"blocks\n";
        block_centers.row(b) = block_list[b]->coordinates().transpose();
    }
    Eigen::MatrixXf test_pts = slam::gen_test_pts(block_centers,blockSide,voxelsize);
    /**
     * @brief Test point set includes 
     * 
     */
    D.push_back(test_pts(Eigen::placeholders::all,Eigen::seqN(0,2)));
    std::cout<<"==========Generated test points ===========\n";
    /**
     * D = {X_train, X_m, F_train, F_m, X_test}
     **/
    SparseGp sgp;  
    std::cout<<"======Prediction starts=======\n";  
    auto tic = std::chrono::high_resolution_clock::now();
    D.push_back(test_pts(Eigen::placeholders::all,Eigen::seqN(0,2)));
    sgp.posterior(D);
    auto toc = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::seconds>(toc-tic);
    std::cout<<"==========completed prediction in "<<duration.count()<<"seconds========\n";

    // /**
    //  * D = {X_train, X_m, F_train,F_m, X_test, mu_t,covar_t}
    //  **/
    // for(auto i:D)     sgp.save_data(i.transpose(),"plot.csv");

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
        
        std::pair<Eigen::MatrixXf, std::vector<float>> temp = slam::gen_pseudo_pts(dataPoint.second);

        std::vector<Eigen::MatrixXf> D;
        /**<Extracting middle of pseudo grid i.e, Laser hit point*/
        Eigen::MatrixXf X_m = temp.first;
        Eigen::MatrixXf X_train = X_m(Eigen::seq(4,Eigen::placeholders::last,9),Eigen::seqN(0,2));
        D.push_back(X_train);
        //Storing pseudo points
        D.push_back(X_m(Eigen::placeholders::all,Eigen::seqN(0,2)));

        //Storing hit points SDF values
        Eigen::Vector<float,180> boundary(Eigen::VectorXf::Zero(180));
        D.push_back(boundary);

        // //Storing pseudo points SDF values
        std::vector<float> sdf_vals = temp.second;
        Eigen::Vector<float,180*9> pseudoPts_sdfVals(sdf_vals.data());
        D.push_back(pseudoPts_sdfVals);

        
        
        // Update map
        float voxelsize =  volume_._dim/volume_._size;

        // unsigned int num_vox_per_pt = volume_._dim/((se::VoxelBlock<FieldType>::side)*voxelsize);
        unsigned int num_vox_per_pt = 1;
        size_t total = num_vox_per_pt * 180 * num_pseudo_pts_;
        allocation_list_.reserve(total);
        unfiltered_alloc_list_.reserve(total);
        keycount_per_block_.reserve(total);
        prev_alloc_list_.reserve(total);
        prev_unfiltered_alloc_list_.reserve(total);
        prev_keycount_per_block_.reserve(total);
        
        int prev_alloc_size = -1;
        update_count_++;
        std::cout<< update_count_ <<"th update"<<'\t'<<"--------------"<<'\n';

        unsigned int allocated = buildAllocationList(allocation_list_.data(), allocation_list_.capacity(), prev_alloc_list_.data(),
                                    sdf_vals.data(), X_m, *volume_._map_index, 180*9, prev_alloc_size,
                                    volume_._size, voxelsize, 2*mu_);
       
        struct less_than_key {
            inline bool operator() (const pointVals<se::key_t>& struct1, const pointVals<se::key_t>& struct2) {
                if(struct1.typeAlloc!=struct2.typeAlloc) return struct1.typeAlloc<struct2.typeAlloc;
                return struct1.hash < struct2.hash;
            }
        };

        #if defined(_OPENMP) && !defined(__clang__)
            __gnu_parallel::sort(allocation_list_.data(),allocation_list_.data() + allocated);
            __gnu_parallel::sort(prev_alloc_list_.data(),prev_alloc_list_.data() + prev_alloc_size);
        #else
            std::sort(prev_alloc_list_.data(),prev_alloc_list_.data() + prev_alloc_size, 
                                            [](const auto& i, const auto& j) {return i.hash < j.hash;});

            std::sort(allocation_list_.data(),allocation_list_.data() + allocated, 
                                            [](const auto& i, const auto& j) {return i.hash < j.hash;});

        #endif
        
        for(unsigned int i=0;i<allocated;i++){
            unfiltered_alloc_list_[i] = allocation_list_[i];
        }
        for(unsigned int i=0;i<prev_alloc_size;i++){
            prev_unfiltered_alloc_list_[i] = prev_alloc_list_[i];
        }
        int num_elem = se::algorithms::filter_ancestors(allocation_list_.data(), allocated, log2(volume_._size),
                                                keycount_per_block_.data());
        int prev_num_elem = se::algorithms::filter_ancestors(prev_alloc_list_.data(), prev_alloc_size, log2(volume_._size),
                                                prev_keycount_per_block_.data());
        for(unsigned int i=0;i<num_elem;i++) {
            std::cout<<allocation_list_[i].hash<<'\t'<<keycount_per_block_[i]<<"\t";
        }
        for(unsigned int i=0;i<allocated;i++) std::cout<<unfiltered_alloc_list_[i].hash<<"\t";
        std::cout<<"----------Prevs\n";
        for(unsigned int i=0;i<prev_num_elem;i++) {
            std::cout<<prev_alloc_list_[i].hash<<'\t'<<prev_keycount_per_block_[i]<<"\t";
        }
        for(unsigned int i=0;i<prev_alloc_size;i++) std::cout<<prev_unfiltered_alloc_list_[i].hash<<"\t";
        std::cout<<"----\n";
        
        std::cout<<"total number of new block points"<<allocated<<"\n";
        std::cout<<"new blocks to be allocated"<<num_elem<<"\n";
        std::cout<<"Prev_num_blocks"<<prev_alloc_size<<"\n";
        std::cout<<"prev num blocks to be allocated"<<prev_num_elem<<"\n";
        // int prev_num_blocks = 0;
        // for(unsigned int i=0;i<prev_num_elem;i++) {
        //     if(prev_unfiltered_alloc_list_[i].typeAlloc==0) {
        //         prev_num_blocks++;
        //     }
        //     else break;
        // }
        // std::cout<<"Prev_num_blocks only"<<prev_num_blocks<<"\n";
        std::vector<se::key_t> keys;

        // for(unsigned int i=0;i<num_elem;i++) {
        //     allocation_list_[i].typeAlloc =0;
        // }
        
        
        for(unsigned int i =0; i < num_elem;i++){
            keys.push_back(allocation_list_[i].hash);
        }
        // for(int i=0;i<allocated;i++)  std::cout<<allocation_list_[i].typeAlloc<<" ";
        
        volume_._map_index->allocate(keys.data(), num_elem);


        slam::predict(D, volume_._map_index,num_elem, voxelsize);

        return 0;
        // const unsigned block_scale = log2(volume_._size) - se::math::log2_const(se::VoxelBlock<FieldType>::side);
        // const float inverseVoxelSize = 1/voxelsize;
        // for(int i=0;i<allocated;i++) {
        //     Eigen::Vector3f voxelPos = unfiltered_alloc_list_[i].pt;
        //     // Eigen::Vector3f voxelPos = allocation_list_[i].pt;

        //     Eigen::Vector3f voxelScaled = (voxelPos * inverseVoxelSize).array().floor();
        //     Eigen::Vector3i voxel = voxelScaled.cast<int>();

        //     se::VoxelBlock<FieldType> * n = volume_._map_index->fetch(voxel.x(), voxel.y(), voxel.z());
        //     // if(n->isNotBlock == 0) ordered_alloc_list_[i].typeAlloc = 0;

        // }
       

        // std::sort(unfiltered_alloc_list_.data(),unfiltered_alloc_list_.data() + allocated, less_than_key());
        // std::sort(allocation_list_.data(),allocation_list_.data() + allocated, less_than_key());

        struct sdf_update funct(mu_, maxWeight_);
        se::functor::projective_map(*volume_._map_index, funct, unfiltered_alloc_list_.data(), prev_unfiltered_alloc_list_.data(),
                                    keycount_per_block_.data(), prev_keycount_per_block_.data(),
                                    num_elem, prev_num_elem, prev_alloc_size -prev_num_elem);
        
    }
    return true;
}