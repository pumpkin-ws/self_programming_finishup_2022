#include "Eigen/Core"
#include "Eigen/Dense"
#include <iostream>

int main(int argc, char** argv) {
    /* For a rotation vector, the norm is the angle of rotation,
        and the unit vector is the direction of rotation.
     */
    Eigen::Vector3d vd{1, 1, 1};
    printf("---------------\n");
    std::cout << vd.norm() << std::endl;
    printf("---------------\n");
    std::cout << vd.normalized() << std::endl;
    printf("---------------\n");
    std::cout << vd << std::endl;
    // Eigen::AngleAxisd aad{}

}