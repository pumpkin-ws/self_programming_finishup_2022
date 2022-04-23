#include "glog/logging.h"
#include "boost/filesystem.hpp"
#include <iostream>

using google::InitGoogleLogging;
using google::SetLogDestination;
using google::INFO;
using google::ERROR;
using google::WARNING;
using google::FATAL;

int main(int argc, char** argv) {
    InitGoogleLogging(argv[0]);
    std::string dir_name = "./log/log_";
    if(!boost::filesystem::exists(dir_name)) {
        boost::filesystem::create_directories(dir_name);
    }
    SetLogDestination(google::INFO, dir_name.c_str());
    char* home = getenv("HOME");
    std::cout << "The home directory is " << home << std::endl;
    LOG(INFO) << "Loggin some information.";
    return EXIT_SUCCESS;
}