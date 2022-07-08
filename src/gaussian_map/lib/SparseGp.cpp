#include "gaussian_map/SparseGp.hpp"

SparseGp::SparseGp(){
    std::ofstream fd(SparseGp::fname_,std::ios::out);
    if (fd.is_open()) fd<<SparseGp::dim_<<'\n';
}

SparseGp::SparseGp(std::string fname) : fname_(fname){
    std::ofstream fd(SparseGp::fname_,std::ios::out);
    if (fd.is_open()) fd<<SparseGp::dim_<<'\n';
}

SparseGp::SparseGp(unsigned int dim, unsigned int train_size, unsigned int pseudo_size, 
                     unsigned int test_size, std::pair<int,int> limit, std::string fname) : 
                     dim_(dim), train_size_(train_size), pseudo_size_(pseudo_size), 
                     test_size_(test_size), limit_(limit), fname_(fname){
   

    std::ofstream fd(SparseGp::fname_,std::ios::out);
    if (fd.is_open()) fd<<SparseGp::dim_<<'\n';
}




std::vector<Eigen::MatrixXf> SparseGp::gen_data(){
    // generate training samples
    Eigen::MatrixXf X_train = gen_samples(SparseGp::limit_, SparseGp::train_size_, SparseGp::dim_);
    // generate Pseudo  points
    Eigen::MatrixXf X_m = gen_pseudo_pts(SparseGp::limit_, SparseGp::pseudo_size_, SparseGp::dim_);
    // Generate training and pseudo targets
    Eigen::MatrixXf F_train,F_m;
    /**
    * COS/SINE
    */
    if(dim_==2) {
        F_train = X_train.cwiseAbs2().rowwise().sum().array().cos();
        F_m = X_m.cwiseAbs2().rowwise().sum().array().cos();
    } 
    else {
        F_train = X_train.array().cos();
        F_m = X_m.array().cos();
    }
    // Generate test samples
    Eigen::MatrixXf X_test = gen_samples(SparseGp::limit_, SparseGp::test_size_, SparseGp::dim_);
    return {X_train, X_m, F_train, F_m, X_test};
    

}

 Eigen::MatrixXf  SparseGp::gen_samples(std::pair<int,int> limit, const int size_,const int dim_){
    
    std::vector<std::default_random_engine> gen(dim_);
    std::uniform_real_distribution<double> unigen(limit.first,limit.second);
    Eigen::MatrixXf X_s(size_,dim_);
    for(unsigned int i=0;i<dim_;i++){
        gen.at(i).seed(i+1);
        for(unsigned int j=0;j<size_;j++)  X_s(j,i) = unigen(gen.at(i));
    }
    return X_s; 

}

Eigen::MatrixXf SparseGp::gen_pseudo_pts(const std::pair<int,int> limit, const unsigned int size, const unsigned int dim){
        
        Eigen::MatrixXf X_m(size,dim);
        Eigen::VectorXf lin_vec(Eigen::VectorXf::LinSpaced(size,-5,5));
        for(unsigned int i=0;i<dim;i++)    X_m.col(i) = lin_vec;
        return X_m;

}


Eigen::MatrixXf SparseGp::kernel(Eigen::MatrixXf X1,Eigen::MatrixXf X2 )  {

    Eigen::MatrixXf M =X1.cwiseAbs2().rowwise().sum().col(0).replicate(1,X2.rows());
    Eigen::RowVectorXf N = X2.cwiseAbs2().rowwise().sum().col(0);
    // std::cout<<"M+Nsize\n"<<(M.rowwise()+N.transpose()).rows()<<"x"<<(M.rowwise()+N.transpose()).cols()<<'\n';
    return  pow(SparseGp::sigma_f_,2) * ( (M.rowwise()+N - 2 * X1*X2.transpose())  / -2*pow(SparseGp::l_,2) ).array().exp();                                ;
}

void SparseGp::posterior(std::vector<Eigen::MatrixXf> &D){
    
    /**
     * D = {X_train, X_m, F_train, F_m, X_test}
     **/
    Eigen::MatrixXf Knn = SparseGp::kernel(D.at(0),D.at(0));
    Eigen::MatrixXf Kmm = SparseGp::kernel(D.at(1),D.at(1));
    Eigen::MatrixXf Kmn = SparseGp::kernel(D.at(1),D.at(0));
    Eigen::MatrixXf Kmt = SparseGp::kernel(D.at(1),D.at(4));
    Eigen::MatrixXf Ktt = SparseGp::kernel(D.at(4),D.at(4));
    Eigen::LLT<Eigen::MatrixXf> L_Kmm;
    L_Kmm.compute(Kmm);

    //Not used  
    //Eigen::MatrixXf pseudo_mu= Kmn.transpose()*L_Kmm.solve(F_m);
    //IID assumption
    Eigen::VectorXd lambda = (Knn.diagonal() - (Kmn.transpose()*L_Kmm.solve(Kmn)).diagonal()).cast<double>();
     /**
     * @brief Add noise
     */
    // Eigen::VectorXf sigma_v = Eigen::VectorXf::Constant(lambda.size(),0.01);
    Eigen::VectorXd sigma_v = Eigen::VectorXd::Zero(lambda.size());

    lambda += sigma_v;
    Eigen::VectorXf lambda_inv = lambda.cwiseInverse().cast<float>();

   
    Eigen::MatrixXf Qm = Kmm + Kmn * lambda_inv.asDiagonal()*Kmn.transpose();
    
    
    Eigen::LLT<Eigen::MatrixXf> L_Qm;
    L_Qm.compute(Qm);
    Eigen::MatrixXf mu_t = Kmt.transpose()*L_Qm.solve(Kmn*lambda_inv.asDiagonal()*D.at(2)); // D.at(2) = F_train
    D.push_back(mu_t);
    Eigen::VectorXf covar_t = Ktt.diagonal() -  (Kmt.transpose()*(L_Kmm.solve(Kmt) - L_Qm.solve(Kmt))).diagonal() ;
    D.push_back(covar_t);

}



void SparseGp::save_data(Eigen::MatrixXf M, std::string filename ){

    /*
     * Saves Eigen Vector "v" in the file specified in "filename" in CSV format.
     */
    const static Eigen::IOFormat CSVFormat(Eigen::FullPrecision, Eigen::DontAlignCols, " ");

	std::ofstream fd(filename,std::ios::app);
	if (fd.is_open()){
        fd << M.format(CSVFormat);
        fd <<'\n';
		fd.close();
	}
}


// int main(){
    
    
//     SparseGp sgp;
//     std::vector<Eigen::MatrixXf> D = sgp.gen_data();
//     /**
//      * D = {X_train, X_m, F_train, F_m, X_test}
//      **/
//     sgp.posterior(D);
//     /**
//      * D = {X_train, X_m, F_train,F_m, X_test, mu_t,covar_t}
//      **/
//     for(auto i:D)     sgp.save_data(i.transpose(),"plot.csv");
    
//     return 0;
// }