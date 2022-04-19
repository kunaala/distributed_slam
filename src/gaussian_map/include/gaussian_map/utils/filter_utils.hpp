#include <iostream>
#include <fstream>
std::fstream filter_data(std::string infile, std::string outfile){
    std::ifstream fd1(infile,std::ios::in);
    std::fstream fd2(outfile,std::ios::out);

    if(fd1.is_open()) {
        std::string test;
        while (!fd1.eof()){
            std::getline(fd1,test,' ');
            if (test.find("FLASER") != std::string::npos){
                std::getline(fd1,test);
                // for (auto i : msg) std::cout<<i<<'\n';
                fd2<<test;
                fd2<<'\n';
            }
            else if (test.find("ODOM") != std::string::npos){
                std::getline(fd1,test);
                // for (auto i : msg) std::cout<<i<<'\n';
                fd2<<test;
                fd2<<'\n';
            }
        }

        
        fd1.close();
        fd2.close();
    }

    else std::cerr<<"file not found"<<'\n';
    return fd2;
}

std::vector<std::string> split_line(std::string str){
  std::vector<std::string> v;
  std::string word=""; 
  for(auto i : str){
    if(i == ' ') {
      v.push_back(word);
      word="";
    }
    else word = word+i;
  }
  return v;
}
std::vector<double> slice_data(std::string str){
  std::vector<double> v;
  std::string word=""; 
  

  for(auto i : str){
    if(i == ' ') {
      try{
        v.push_back(std::stod(word));
        word="";
      }
      catch(...){}
    }
    else word = word+i;
  }
  return v;
}
