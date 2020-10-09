#pragma once
#include <string>
#include <iostream>
#include <fstream>
#include <experimental/filesystem>

namespace ARDL {
    namespace Util {
        static std::string getModelFromGazeboPath(std::string input, std::string modelUrdf= "model.urdf") {
            std::string output= "";
            std::ifstream f(input.c_str());
            if(!f.good()) {
                std::string model_paths(getenv("GAZEBO_MODEL_PATH"));
                std::string split= ":";
                size_t last= 0, next= 0;
                std::cout<<model_paths<<std::endl;
                while((next= model_paths.find(split, last)) != std::string::npos || last==0) {
                    std::string model_path= model_paths.substr(last, next - last);
                std::cout<<model_path<<std::endl;
                    std::experimental::filesystem::path models{model_path + "/"+input+"/"};
                std::cout<<model_path<< "/"<<input<<"/"<<std::endl;
                    if(std::experimental::filesystem::exists(models)) {
                        if(std::ifstream(model_path + "/" + input + "/"+modelUrdf).good()) {
                            output= model_path + "/" + input + "/"+modelUrdf;
                            std::cout<<output<<std::endl;
                            break;
                        } else {
                            std::cerr << "FAILED TO GET MODEL FROM FAILED" << model_path + "/" + input + "/"+modelUrdf
                                      << std::endl;
                        }
                    } else {
                        std::cerr << "FAILED TO GET MODEL PATH : PATH DOESN'T EXIST" << models << std::endl;
                    }

                    last= next + 1;
                }
                if(std::string::npos == last) {
                    std::cerr << "FAILED TO GET MODEL PATH cehck" << input << std::endl;
                    exit(-1);
                }
            }else{
                output = input;
                std::cout<<"FAILED: "<<output<<std::endl;
            }
            return output;
        }
    } // namespace Util
} // namespace ARDL
