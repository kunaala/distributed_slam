#include<gaussian_map/dataloader.h>
#include<string>

dataloader::dataloader(std::string datafile, int type) {
    inpFile_.open("intel.gfs.log");
    type_ = type;
}

std::pair<int,Eigen::VectorXf> dataloader::getNextPoint() {
    std::string line;
    std::getline(inpFile_, line);
    if(inpFile_.eof()) {
        return std::make_pair(-1, Eigen::Vector2f::Zero());
    }
    int typeR = 0;
    std::string token;
    std::string delimiter = " ";
    size_t pos = 0;
    int num_readings = 3;
    pos = line.find(delimiter);

    token = line.substr(0, pos);
    line.erase(0, pos + delimiter.length());

    if(token=="ODOM") typeR = 0;
    else if(token=="FLASER") {
        typeR = 1;
        pos = line.find(delimiter);
        token = line.substr(0, pos);
        num_readings = stoi(token);
        line.erase(0, pos + delimiter.length());
    }
    Eigen::VectorXf input(num_readings);
    unsigned int l=0;
    while ((pos = line.find(delimiter)) != std::string::npos && num_readings--) {
        token = line.substr(0, pos);
        
        input[l] = stof(token);
        line.erase(0, pos + delimiter.length());
        l++;
    }
    return std::make_pair(typeR,input);
}