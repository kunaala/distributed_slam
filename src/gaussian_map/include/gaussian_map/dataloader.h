#include<iostream>
#include<fstream>
#include<vector>
#include<Eigen/Dense>

class dataloader {
    protected:
        std::string dataFile_;
        std::ifstream inpFile_;
        int type_;

    public:

        /**
         * Constructor using the datafile location and type of dateset
         * 
         * \param[in] datafile String denoting the location of the datafile
         * \param[in] type indicating the type of data. Irrelevant
         * right now, but will use in future for loading different
         * type of datesets using this class.
         */
        dataloader(std::string datafile, int type);

        /**
         * Gets the next point according to the timestamp or the line in the dataset
         *
         * \return Returns a tuple having the type(0,1 -> ODOM, FLASER) of data point
         * and the datapoint values.
         */
        std::pair<int,Eigen::VectorXf> getNextPoint();
};