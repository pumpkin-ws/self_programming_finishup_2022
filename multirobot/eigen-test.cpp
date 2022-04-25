#include "Eigen/Core"
#include "Eigen/Dense"
#include <iostream>

int main(int argc, char** argv) {
    Eigen::Vector3d vd{1, 1, 1};
    printf("---------------\n");
    std::cout << vd.norm() << std::endl;
    printf("---------------\n");
    std::cout << vd.normalized() << std::endl;
    printf("---------------\n");
    std::cout << vd << std::endl;
    // Eigen::AngleAxisd aad{}

}