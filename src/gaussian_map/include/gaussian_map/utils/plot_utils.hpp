#include <iostream>
#include <fstream>
#include<Eigen/Core>
using namespace Eigen;

void save_data(Eigen::MatrixXd m, std::string filename ){

    /*
     * Saves Eigen map "m" in the file specified in "filename" in CSV format.
     */
    const static Eigen::IOFormat CSVFormat(FullPrecision, DontAlignCols, ", ");

	std::ofstream fd(filename,std::ios::out);
	if (fd.is_open()){
		fd << m.format(CSVFormat);
        fd <<'\n';
		fd.close();
	}
}

std::vector<int8_t> getDistGrid(Eigen::MatrixXd X_test, Eigen::VectorXd Y_mean ,double resolution, double xLimit, double yLimit) {
    std::vector<int8_t> grr(6000*6000,-1);
    int numX = xLimit/resolution;
    int numY = yLimit/resolution;
    int num = X_test.rows();
    std::cout<<num<<"\n";
    double yMin = 0.0, yMax = 0.0;
    for(int i=0;i<num;i++) {
        yMin = std::min(yMin,Y_mean(i));
        yMax = std::max(yMax,Y_mean(i));
    }
    std::cout<<yMin<<" "<<yMax<<"\n";
    for(int i=0;i<num;i++) {
        grr[int(X_test(i,0) + numX)*6000 + int(X_test(i,1) + numY)] = int8_t(100*(Y_mean(i) - yMin)/(yMax-yMin));
    }
    return grr;
}