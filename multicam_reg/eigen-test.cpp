#include "Eigen/Dense"
#include "Eigen/Core"
#include <iostream>
#include <thread>

int main(int argc, char** argv) {
    Eigen::Matrix4f id;
    std::cout << id.size() << std::endl;
    std::cout << id.row(0)[3] << std::endl;
}