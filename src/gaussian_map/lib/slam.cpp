#include <gaussian_map/slam.h>

slam::slam(const Eigen::Vector3f vol_res, const Eigen::Vector3i vol_dim, std::string datafile, float ps_grid_res, float mu, ros::NodeHandle &nh) : agentT_(Eigen::Matrix4f::Identity()), dataseq_(datafile, 0), nh_(nh)
{
    volume_dimension_ = vol_dim;
    volume_resolution_ = vol_res;
    dataFile_ = datafile;
    ps_grid_res_ = ps_grid_res;
    mu_ = mu;

    discrete_vol_ptr_ = std::make_shared<se::Octree<FieldType>>();
    discrete_vol_ptr_->init(volume_resolution_.x(), volume_dimension_.x());
    volume_ = Volume<FieldType>(volume_resolution_.x(), volume_dimension_.x(), discrete_vol_ptr_.get());
}

/**
 * @brief Retrieves SDF value at *voxelPos* from the octree *map_index*
 *
 * @param map_index
 * @param voxelPos Matrix with rows as voxel locations
 * @param vector of sdf_data for all voxelPos
 * @return Matrix with first column as sdf values of voxelPos and second column 
 * as their corresponding occurences
 */
Eigen::MatrixXf slam::retrieve_sdf(se::Octree<FieldType> *map_index, Eigen::MatrixX3f voxelPos)
{

    Eigen::MatrixXf sdf_data(voxelPos.rows(), 2);
    Eigen::Vector3f voxelScaled;
    Eigen::Vector3i voxel;
    for (unsigned int i = 0; i < voxelPos.rows(); i++)
    {
        voxelScaled = (voxelPos.row(i) * (1/ voxel_size_)).array().floor();
        voxel = voxelScaled.cast<int>();
        // for(unsigned int i=0;i<prev_alloc_size_;i++){
        //     if (prev_alloc_list_[i].pt == voxel) std::cout<<"matched pt "<<voxel<<"in prev alloc list\n";
        // }
        // for(unsigned int i=0;i<allocated_;i++){
        //     if (allocation_list_[i].pt == voxel) std::cout<<"matched pt "<<voxel<<"in current alloc list\n";
        // }
        se::VoxelBlock<FieldType>::value_type temp_data;

        // To generate mu_0 for first occurences
        // std::random_device seed;
	    // std::mt19937 gen(seed());
	    // std::uniform_real_distribution<> dist(0,-(mu_));

        se::VoxelBlock<FieldType> *block = map_index->fetch(voxel.x(), voxel.y(), voxel.z());
        // check if block is allocated for the voxel
        if (block)
        {
            temp_data = block->data(voxel);
            std::cout<<"ALREADY ALLOCATED VOXEL\n";
            sdf_data(i, 0) = temp_data.x;
            sdf_data(i, 1) = temp_data.y;
            std::cout<<"\nblock:"<<block->coordinates()<<"\tvoxel:"<<voxel<<"\toccurence: "<<temp_data.y<<'\n';

        }
        else
        {
            std::cout<<"new_voxel\n";
            sdf_data(i, 0) = mu_;
            sdf_data(i, 1) = 1;        
        }
    }
    return sdf_data;
}

// Eigen::MatrixXf gen_test_pts(se::Octree<FieldType> *map_index, unsigned int num_elem, float voxel_size_)
// {
//     /**
//      * @brief Generates test points (including hit and pseudo points from the newly allocated blocks/Octants)
//      *
//      */
//     std::vector<se::VoxelBlock<FieldType> *> block_list;
//     const unsigned int blockSide = map_index->blockSide;
//     block_list.reserve(num_elem);
//     map_index->getBlockList(block_list, 0);
//     Eigen::MatrixXi block_centers(num_elem, 3);
//     for (unsigned int b = 0; b < num_elem; b++)
//     {
//         block_centers.row(b) = block_list[b]->coordinates().transpose();
//     }
//     /**
//      * @brief Test point set includes hit points and pseudo points as
//      * 1. To avoid discontinuities  due to  local kriging , the newly allocated blocks are pooled together
//      *  as training and pseudo datasets
//      * 2. The prediction model is expected to predict sdf values of hit and pseudo points based on its neigbouring blocks
//      *     creating smooth continuous surfaces
//      */
//     Eigen::MatrixXf test_pts(block_centers.rows() * blockSide * blockSide, 3);
//     Eigen::VectorXf lin_v(Eigen::VectorXf::LinSpaced(blockSide + 1, -(blockSide * voxel_size_ / 2), blockSide * voxel_size_ / 2));

//     for (unsigned int c = 0; c < block_centers.rows(); c++)
//     {
//         uint64_t b_x = block_centers(c, 0), b_y = block_centers(c, 1), b_z = block_centers(c, 2);
//         // std::cout<<"\n In block with Block center: "<<block_centers.row(c)<<'\t'<<" and Block Hash: "<<map_index->hash(x, y, z)<<'\n';

//         for (unsigned int i = 0; i < blockSide; i++)
//         {
//             for (unsigned int j = 0; j < blockSide; j++)
//             {
//                 Eigen::RowVector3f pt;
//                 pt << b_x + lin_v[i], b_y + lin_v[j], b_z + 0.f;
//                 test_pts.row(c * blockSide * blockSide + i * blockSide + j) = pt;
//             }
//         }
//     }
//     std::cout << "==========Generated " << test_pts.rows() << " test points ===========\n";

//     return test_pts;
// }



void slam::visualize(const std::vector<Eigen::MatrixXf> &D){
   /**
    * @brief To visualize the predicted sdf values
    * 
    */
    // Eigen::MatrixXf X_pred = D.at(0);
    // Eigen::MatrixXf map(6000, 6000);
    // float map_res = 0.1f;
    // int off_x = 3000;
    // int off_y = 3000;
    // for (unsigned int p = 0; p < X_pred.rows(); p++)
    // {
    //     map(int(X_pred(p, 0) / map_res + off_x), int(X_pred(p, 1) / map_res + off_y)) = D.at(1)(p);
    // }
    std::string viz_file = "";
    std::string count_str = std::to_string(update_count_);
    viz_file += count_str + "_" + slam::fname_;
    std::cout<<"Saving prediction to: "<<viz_file<<'\n';
    //predicted points - x,y coordinates
    save_data(D.at(0).transpose(), viz_file);
    // predicted points sdf values
    save_data(D.at(1).transpose(), viz_file);

}

std::vector<int8_t> slam::gen_grid(std::vector<Eigen::MatrixXf> &D)
{

    /**
     * D = {X_pred, mu_t,covar_t}
     **/
    std::vector<int8_t> flat_map(map_size_.first * map_size_.second, -1);
    int xMin = -map_size_.first/2;
    int yMin = -map_size_.second/2;
    int xMax = map_size_.first/2;
    int yMax = map_size_.first/2;
    int numX = xMax / map_res_;
    int numY = yMax / map_res_;
    Eigen::MatrixXf X_pred = D.at(0);
    for (unsigned int i = 0; i < X_pred.rows(); i++)
    {
        flat_map.at(int(X_pred(i, 0) / map_res_ + numX) * map_size_.first + int(X_pred(i, 1) / map_res_ + numY)) = int8_t(100 * (D.at(1)(i)));
    }
    return flat_map;
}

bool slam::mapNext()
{
    std::pair<int, Eigen::VectorXf> dataPoint = dataseq_.getNextPoint();

    MapPub ros_map(slam::nh_);

    if (dataPoint.first == 0)
    { /**<ODOM datapoint*/
        // Updating robot pose

        Eigen::VectorXf vals = dataPoint.second;
        // Updating pose on Ros map
        ros_map.update_odom(vals);

        Eigen::Matrix3f Rx, Ry, Rz;

        Rx << 1, 0, 0,
            0, cos(vals[2]), -sin(vals[2]),
            0, sin(vals[2]), cos(vals[2]);
        Ry << cos(vals[2]), 0, sin(vals[2]),
            0, 1, 0,
            -sin(vals[2]), 0, cos(vals[2]);
        Rz = Eigen::Matrix3f::Identity();
        Eigen::Vector3f tr(vals[0], vals[1], 0.0);

        Eigen::Matrix4f newPose = Eigen::Matrix4f::Identity();
        newPose.block(0, 0, 3, 3) = Rz * Ry * Rx;
        newPose.block(0, 3, 3, 1) = tr;
        agentT_.setPose(newPose);
    }

    else if (dataPoint.first == 1)
    { // Lidar scan datapoint

        //@@@@@@@@@@@@@@ UPDATE @@@@@@@@@@@@@
        std::cout << "@@@@@@@@@@@@@@@@@ update count : " << update_count_ << '\t' << "@@@@@@@@@@@@@@" << '\n';
        // generate psuedo points and their corresponding sdf values
        std::pair<Eigen::MatrixXf, std::vector<float>> temp = gen_pseudo_pts(dataPoint.second, ps_grid_res_, num_pseudo_pts_);

        /**<Extracting middle of pseudo grid i.e, Laser hit point*/
        Eigen::MatrixXf X_m = agentT_.bodyToWorld(temp.first.transpose()).transpose();
        Eigen::MatrixXf X_train = X_m(Eigen::seq(4, Eigen::placeholders::last, 9), Eigen::placeholders::all);
        // convert to world coordinates
        std::vector<float> pseudo_sdf_vals = temp.second;

        /**<Allocation in Octree begins*/
        voxel_size_ = volume_._dim / volume_._size;
        // unsigned int num_vox_per_pt = volume_._dim/((se::VoxelBlock<FieldType>::side)*voxel_size_);
        unsigned int num_vox_per_pt = 1;
        // Estimating number of blocks to be allocated per LiDAR scan
        size_t total = num_vox_per_pt * 180 * num_pseudo_pts_;
        // reserve memory in allocation lists
        allocation_list_.reserve(total);
        unfiltered_alloc_list_.reserve(total);
        keycount_per_block_.reserve(total);
        prev_alloc_list_.reserve(total);
        prev_unfiltered_alloc_list_.reserve(total);
        prev_keycount_per_block_.reserve(total);

        prev_alloc_size_ = -1;
        allocated_ = buildAllocationList(allocation_list_.data(), allocation_list_.capacity(), prev_alloc_list_.data(),
                                                     pseudo_sdf_vals.data(), X_m, *volume_._map_index, 180 * num_pseudo_pts_,
                                                     prev_alloc_size_, volume_._size, voxel_size_, 2 * mu_);

        /**<sort blocks in allocation list*/
        #if defined(_OPENMP) && !defined(__clang__)
                __gnu_parallel::sort(allocation_list_.data(), allocation_list_.data() + allocated);
                __gnu_parallel::sort(prev_alloc_list_.data(), prev_alloc_list_.data() + prev_alloc_size);
        #else
                std::sort(prev_alloc_list_.data(), prev_alloc_list_.data() + prev_alloc_size_,
                        [](const auto &i, const auto &j)
                        { return i.hash < j.hash; });

                std::sort(allocation_list_.data(), allocation_list_.data() + allocated_,
                        [](const auto &i, const auto &j)
                        { return i.hash < j.hash; });
        #endif

        /**< Store unfiltered allocation lists*/
        for (unsigned int i = 0; i < prev_alloc_size_; i++)
        {
            prev_unfiltered_alloc_list_[i] = prev_alloc_list_[i];
        }

        for (unsigned int i = 0; i < allocated_; i++)
        {
            unfiltered_alloc_list_[i] = allocation_list_[i];
        }

        /**< Filters ancestors to seperate leaves from internal nodes and root*/
        int num_elem = se::algorithms::filter_ancestors(allocation_list_.data(), allocated_, log2(volume_._size),
                                                        keycount_per_block_.data());
        int prev_num_elem = se::algorithms::filter_ancestors(prev_alloc_list_.data(), prev_alloc_size_, log2(volume_._size),
                                                             prev_keycount_per_block_.data());
        std::cout << "\nPreviously allocated list\n";
        for (unsigned int i = 0; i < prev_num_elem; i++)
        {
            std::cout << prev_alloc_list_[i].hash << '\t' << prev_alloc_list_[i].pt <<'\t' << prev_keycount_per_block_[i] << "\t";
        }
        std::cout << "\ncurrent Allocation list \n";
        for (unsigned int i = 0; i < num_elem; i++)
        {
            std::cout << allocation_list_[i].hash << '\t' << allocation_list_[i].pt <<'\t' << keycount_per_block_[i] << "\t";
        }

        /**<allocate block leaves in octree*/
        std::vector<se::key_t> keys;
        for (unsigned int i = 0; i < num_elem; i++){
            keys.push_back(allocation_list_[i].hash);
            allocation_list_[i].typeAlloc =1;
        }
        
        volume_._map_index->allocate(keys.data(), num_elem);

        /**<update sdf values in Octree*/
        struct sdf_update funct(mu_, maxWeight_);
        se::functor::projective_map(*volume_._map_index, funct, unfiltered_alloc_list_.data(), prev_unfiltered_alloc_list_.data(),
                                    keycount_per_block_.data(), prev_keycount_per_block_.data(),
                                    num_elem, prev_num_elem, prev_alloc_size_ - prev_num_elem);

        update_count_++;

        std::cout << "\n==============UPDATION DONE ====================================\n";
        std::cout << "total number of new block points: " << allocated_ << "\n";
        std::cout << "new blocks to be allocated: " << num_elem << "\n";
        std::cout << "Prev_num_blocks: " << prev_alloc_size_ << "\n";
        std::cout << "prev num blocks to be allocated: " << prev_num_elem << "\n";
        std::cout << "\n==================================================\n";
        // int prev_num_blocks = 0;
        // for(unsigned int i=0;i<prev_num_elem;i++) {
        //     if(prev_unfiltered_alloc_list_[i].typeAlloc==0) {
        //         prev_num_blocks++;
        //     }
        //     else break;
        // }

        
        // register training points in D vector for prediction
        if (update_count_ % predict_cycle_ == 0)
        {    
            //@@@@@@@@@@@@@@ PREDICTION @@@@@@@@@@@@@

            std::vector<Eigen::MatrixXf> D;
            D.push_back(X_train(Eigen::placeholders::all, Eigen::seq(0, 1)));
            // TODO : Data centering/Averaging - does not affect TSDF posterior\\


            //@@@@@@@@@@@@@@@@@ USING SPARSE PSEUDO POINT GAUSSIAN PROCESS @@@@@@@@@@@@@
            // Storing pseudo points
            D.push_back(X_m(Eigen::placeholders::all,Eigen::seqN(0,2)));
            /**<Retrieving sdf values of training points from Octree*/
            Eigen::MatrixXf pseudo_sdf_data = retrieve_sdf(volume_._map_index, X_m);
            Eigen::MatrixXf sdf_data = pseudo_sdf_data(Eigen::seq(4, Eigen::placeholders::last, 9), Eigen::placeholders::all);
            std::cout << "\n+++++++++++++++ retrieved sdf of training and pseudo points +++++++++++++\n";
            //sdf_data =[ sdf_values for training points,  their corresponding occurences 'm' ]
            //sdf_data =[ sdf_values for pseudo points,  their corresponding occurences 'm_pseudo' ]
            D.push_back(sdf_data.col(0));
            D.push_back(pseudo_sdf_data.col(0));

            //D = {X_train, X_m, F_train,F_m}

            auto tic = std::chrono::high_resolution_clock::now();
            // Eigen::MatrixXf X_test = gen_test_pts(map_size_, voxel_size_);
            Eigen::MatrixXf X_test = gen_test_pts(map_size_, map_res_);
            Eigen::MatrixXf X_t = X_test(Eigen::placeholders::all, Eigen::seq(0, 1));
            std::cout << "====== test points generated =======\n";
            SparseGp sgp;
            sgp.sparse_posterior(D, X_t);
            //@@@@@@@@@@@@@@@@@ USING SPARSE PSEUDO POINT GAUSSIAN PROCESS @@@@@@@@@@@@@

            
            //@@@@@@@@@@@@@@@@@ USING Precision Matrix Z in GAUSSIAN PROCESS @@@@@@@@@@@@@
            /**<Retrieving sdf values of training points from Octree*/
            /*
            Eigen::MatrixXf sdf_data = retrieve_sdf(volume_._map_index, X_train);
            std::cout << "\n+++++++++++++++ retrieved sdf of training points +++++++++++++\n";
            // sdf_data =[ sdf_values for training points,  their corresponding occurences 'm' ]
            D.push_back(sdf_data.col(0));
            D.push_back(sdf_data.col(1));
            
            //D = {X_train, F_train,m_train}

            auto tic = std::chrono::high_resolution_clock::now();
            // Eigen::MatrixXf X_test = gen_test_pts(map_size_, voxel_size_);
            Eigen::MatrixXf X_test = gen_test_pts(map_size_, map_res_);
            Eigen::MatrixXf X_t = X_test(Eigen::placeholders::all, Eigen::seq(0, 1));
            std::cout << "====== test points generated =======\n";
            SparseGp sgp;

            //D = {X_train, F_train,m_train}

            sgp.posterior(D, X_t);
            */
            //@@@@@@@@@@@@@@@@@ USING Precision Matrix Z in GAUSSIAN PROCESS @@@@@@@@@@@@@

            /**
             *  D = {X_test, mu_t,covar_t}
             */
            slam::visualize(D);
            
            auto toc = std::chrono::high_resolution_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::seconds>(toc - tic);
            std::cout << "\n==============PREDICTION DONE "<<"in "<<duration.count() << "seconds========\n";
            // for(unsigned int i =0; i<D.at(0).rows();i++){
            //     std::cout<<"sdf value predicted for "<<D.at(0).row(i).transpose()<<":\t"<<D.at(1).row(i)<<'\n';
            // }
            // std::cout << "\n==================================================\n";
        }
        
        // std::vector<int8_t> map_pts = slam::gen_grid(D);
        // ros_map.update_map(map_pts);
        // std::this_thread::sleep_for (std::chrono::seconds(1) * 5);

        
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
    }
    return true;
}