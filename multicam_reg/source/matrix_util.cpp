#include "matrix_util.hpp"

cv::Mat eulerAnglesToRotationMatrix(cv::Vec3d& theta, std::string rotation_order) {
    using std::sin; using std::cos;
    cv::Mat R_x = (cv::Mat_<double>(3, 3) << 1,      0,               0,
                                             0,      cos(theta[0]),  -sin(theta[0]),
                                             0,      sin(theta[0]),   cos(theta[0]));

    cv::Mat R_y = (cv::Mat_<double>(3, 3) << cos(theta[1]),   0,   sin(theta[1]),
                                             0,               1,   0,
                                            -sin(theta[1]),   0,   cos(theta[1]));    

    cv::Mat R_z = (cv::Mat_<double>(3, 3) << cos(theta[2]),   -sin(theta[2]),   0,
                                             sin(theta[2]),    cos(theta[2]),   0,
                                             0,                0,               1);
    if (rotation_order == "zyx") {
        return R_x * R_y * R_z;
    } else if (rotation_order == "xyz") {
        return R_z * R_y * R_x;
    } else {
        fprintf(stderr, "wrong rotation order specified, should be xyz or zyx.\n");
        return cv::Mat();
    }                                                                                                                                                           
}

