#include<gaussian_map/slam.h>


slam::slam(const Eigen::Vector3f vol_res, const Eigen::Vector3i vol_dim,std::string datafile, float ps_grid_res, float mu):
            agentT_(Eigen::Matrix4f::Identity()), dataseq_(datafile,0){
    volume_dimension_ = vol_dim;
    volume_resolution_ = vol_res;
    dataFile_ = datafile;
    ps_grid_res_ = ps_grid_res;
    mu_ = mu;
    

    discrete_vol_ptr_ = std::make_shared<se::Octree<FieldType> >();
    discrete_vol_ptr_->init(volume_resolution_.x(), volume_dimension_.x());
    volume_ = Volume<FieldType>(volume_resolution_.x(), volume_dimension_.x(), discrete_vol_ptr_.get());

    
}

/**
 * @brief Retrieves SDF value at *voxelPos* from the octree *map_index*  
 * 
 * @param map_index 
 * @param voxelPos Matrix with rows as voxel locations
 * @param vector of sdf_data for all voxelPos
 * @return se::VoxelBlock<FieldType>::value_type 
 */
std::vector<se::VoxelBlock<FieldType>::value_type> slam::retrieve_sdf(se::Octree<FieldType> *map_index, Eigen::MatrixX3f voxelPos){
    
    std::vector<se::VoxelBlock<FieldType>::value_type> sdf_data;
    for(unsigned int i=0;i<voxelPos.rows();i++){
        Eigen::Vector3f voxelScaled = (voxelPos.row(i) *(1.f/voxel_size_)).array().floor();
        Eigen::Vector3i voxel = voxelScaled.cast<int>();
        se::VoxelBlock<FieldType> *block= map_index->fetch(voxel.x(), voxel.y(), voxel.z());
        if (block){
            VoxelBlockHandler<FieldType> handler = {block, voxel};
            sdf_data.push_back(handler.get());
        }
        else{
            se::VoxelBlock<FieldType>::value_type temp_data;
            temp_data.x = 0.f;
            temp_data.y = 1;
            sdf_data.push_back(temp_data);
        }
    }   
    return sdf_data;	

}


void  slam::predict(std::vector<Eigen::MatrixXf> &D, se::Octree<FieldType> *map_index, unsigned int num_elem, float voxel_size_, 
                        Eigen::VectorXf m){
    std::vector<se::VoxelBlock<FieldType>*> block_list;
    const unsigned int blockSide = map_index->blockSide;
    block_list.reserve(num_elem);
    map_index->getBlockList(block_list,0);
    Eigen::MatrixXi block_centers(num_elem,3);
    for(unsigned int b=0;b<num_elem;b++) {
        block_centers.row(b) = block_list[b]->coordinates().transpose();
    }
    /**
     * @brief Test point set includes hit points and pseudo points as
     * 1. To avoid discontinuities  due to  local kriging , the newly allocated blocks are pooled together
     *  as training and pseudo datasets
     * 2. The prediction model is expected to predict sdf values of hit and pseudo points based on its neigbouring blocks
     *     creating smooth continuous surfaces
     */
    Eigen::MatrixXf X_test = gen_test_pts(block_centers,blockSide,voxel_size_);
    D.push_back(X_test(Eigen::placeholders::all,Eigen::seqN(0,2)));
    std::cout<<"==========Generated "<<X_test.rows()<<" test points ===========\n";
    /**
     * D = {X_train, X_m, F_train, F_m, X_test}
     **/
     
    std::cout<<"======Prediction starts=======\n";  
    auto tic = std::chrono::high_resolution_clock::now();
    SparseGp sgp; 
    sgp.posterior(D,m);
    auto toc = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::seconds>(toc-tic);
    std::cout<<"==========completed prediction in "<<duration.count()<<"seconds========\n";
    /**
     * D = {X_test, mu_t,covar_t}
     **/
    // std::vector<int8_t> flat_map(6000*6000,-1);
    // int xMin =-300;
    // int yMin =-300;
    // int xMax =300;
    // int yMax =300;
    // int numX = xMin/0.1;
    // int numY = yMin/0.1;
    // Eigen::MatrixXf X_pred = D.at(0);
    // for(unsigned int i=0;i<X_pred.rows();i++) {
    //     flat_map[int(X_pred(i,0) + numX)*6000 + int(X_pred(i,1)+numY)] = int8_t(100*(D.at(1)(i)-yMin)/(yMax-yMin));
    // }

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
        
        std::pair<Eigen::MatrixXf, std::vector<float>> temp = gen_pseudo_pts(dataPoint.second, ps_grid_res_, num_pseudo_pts_);

        std::vector<Eigen::MatrixXf> D;
        // Convert pseudo poitns to world coordinates

        Eigen::MatrixXf X_m = temp.first;
        X_m = agentT_.bodyToWorld(X_m.transpose()).transpose();
        // TODO : Data centering/Averaging - does not affect TSDF posterior 
        /**<Extracting middle of pseudo grid i.e, Laser hit point*/



        


        /************WITH PSEUDO POINTS*******************/

        //Storing pseudo points
        // D.push_back(X_m(Eigen::placeholders::all,Eigen::seqN(0,2)));

        //Storing hit points SDF values
        // Eigen::Vector<float,180> boundary(Eigen::VectorXf::Zero(180));
        // D.push_back(boundary);

        // //Storing pseudo points SDF values
        std::vector<float> sdf_vals = temp.second;
        // Eigen::Vector<float,180*9> pseudoPts_sdfVals(sdf_vals.data());
        // D.push_back(pseudoPts_sdfVals);
        /************WITH PSEUDO POINTS*******************/
        

        
        
        // Update map
        voxel_size_ =  volume_._dim/volume_._size;

        // unsigned int num_vox_per_pt = volume_._dim/((se::VoxelBlock<FieldType>::side)*voxel_size_);
        unsigned int num_vox_per_pt = 1;
        size_t total = num_vox_per_pt * 180 * num_pseudo_pts_;
        allocation_list_.reserve(total);
        unfiltered_alloc_list_.reserve(total);
        keycount_per_block_.reserve(total);
        prev_alloc_list_.reserve(total);
        prev_unfiltered_alloc_list_.reserve(total);
        prev_keycount_per_block_.reserve(total);
        predicted_keycount_per_block_.reserve(total);
        
        int prev_alloc_size = -1;
        update_count_++;
        std::cout<< update_count_ <<"th update"<<'\t'<<"--------------"<<'\n';
        unsigned int allocated = buildAllocationList(allocation_list_.data(), allocation_list_.capacity(), prev_alloc_list_.data(),
                                    sdf_vals.data(), X_m, *volume_._map_index, 180*num_pseudo_pts_, prev_alloc_size,
                                    volume_._size, voxel_size_, 2*mu_);
       
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
        std::cout<<"Allocation list before prediction--------------\n";
        for(unsigned int i=0;i<num_elem;i++) {
            std::cout<<allocation_list_[i].hash<<'\t'<<keycount_per_block_[i]<<"\t";
        }
        // for(unsigned int i=0;i<allocated;i++) std::cout<<unfiltered_alloc_list_[i].hash<<"\t";
        std::cout<<"----------Previously allocated list \n";
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

        //Retrieving sdf values
        // Add Z dimension
        Eigen::MatrixXf X_train = X_m(Eigen::seq(4,Eigen::placeholders::last,9),Eigen::seqN(0,2));
        D.push_back(X_train);
        Eigen::MatrixXf X_trainer(X_train.rows(),3);
        X_trainer.col(0) = X_train.col(0);
        X_trainer.col(1) = X_train.col(1);
        X_trainer.col(2) = Eigen::VectorXf::Zero(X_train.rows());
        std::vector<se::VoxelBlock<FieldType>::value_type> sdf_data;
        
        sdf_data = retrieve_sdf(volume_._map_index, X_trainer);
        Eigen::VectorXf F_train(X_train.rows()),m(X_train.rows());

        for(unsigned int i=0;i<X_train.rows();i++){
            F_train(i) = sdf_data.at(i).x;
            m(i) = sdf_data.at(i).y;
        }
        D.push_back(F_train);

        slam::predict(D, volume_._map_index,num_elem, voxel_size_, m);
        
        /**
         * @brief Allocate predicted points in Octree
         *  D = {X_test, mu_t,covar_t}
         */
        new_allocation_list_.reserve(0);
        unsigned int num_predicted = D.at(1).size();
        size_t prediction_size = num_vox_per_pt * num_predicted;
        pred_allocation_list_.reserve(prediction_size);
        std::vector<float> new_sdf_vals(num_predicted);
        Eigen::VectorXf::Map(&new_sdf_vals[0], D.at(1).size()) = D.at(1);
        Eigen::MatrixXf X_predicted(num_predicted,3);
        X_predicted.col(0) = D.at(0).col(0);
        X_predicted.col(1) = D.at(0).col(1);
        X_predicted.col(2) = Eigen::VectorXf::Zero(num_predicted);

        int allocated_before = -1;

        unsigned int new_allocated = buildAllocationList(new_allocation_list_.data(), new_allocation_list_.capacity(), pred_allocation_list_.data(),
                                    new_sdf_vals.data(),X_predicted, *volume_._map_index, num_predicted, allocated_before,
                                    volume_._size, voxel_size_, 2*mu_);
        std::cout<<"blocks updated after prediction: \t"<< allocated_before<<'\n';
        std::cout<<"after prediction, BEFORE filtering: \n";
        for(unsigned int i=0;i<allocated_before;i++) {
            std::cout<<pred_allocation_list_[i].hash<<'\t';
        }
        std::cout<<'\n';
        std::sort(pred_allocation_list_.data(),pred_allocation_list_.data() + allocated_before, 
                                            [](const auto& i, const auto& j) {return i.hash < j.hash;});
        int predicted_num_elem = se::algorithms::filter_ancestors(pred_allocation_list_.data(), allocated_before, log2(volume_._size),
                                                predicted_keycount_per_block_.data());
        // std::cout<<"after prediction, After filtering: \n";
        // for(unsigned int i=0;i<allocated_before;i++) {
        //     std::cout<<allocation_list_[i].hash<<'\t'<<predicted_keycount_per_block_[i]<<"\t";
        // }
        // std::cout<<'\n';
        return 0;
        //***********************************************//
        
        // const unsigned block_scale = log2(volume_._size) - se::math::log2_const(se::VoxelBlock<FieldType>::side);
        // const float inversevoxel_size_ = 1/voxel_size_;
        // for(int i=0;i<allocated;i++) {
        //     Eigen::Vector3f voxelPos = unfiltered_alloc_list_[i].pt;
        //     // Eigen::Vector3f voxelPos = allocation_list_[i].pt;

        //     Eigen::Vector3f voxelScaled = (voxelPos * inversevoxel_size_).array().floor();
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