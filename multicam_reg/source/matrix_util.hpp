#ifndef MAX_UTIL_HPP_
#define MAX_UTIL_HPP_
#include "opencv2/opencv.hpp"

/**
 * @brief For aubo robot, the rotation order is zyx; for ur, the rotation order is xyz. The rotation should be roll,
 * pitch, yaw rotations following the Tait-Bryan rotations, and Tait-Bryan rotation is an intrinsic rotation, meaning the 
 * 
 * 
 * @param theta the euler angle
 * @param rotation_order the rotation order should be either xyz or zyx
 * @return cv::Mat 
 */
cv::Mat eulerAnglesToRotationMatrix(cv::Vec3d& theta, std::string rotation_order = "zyx");

bool isRotationMatrix(cv::Mat &R) {
    cv::Mat Rt;
    cv::transpose(R, Rt);
    cv::Mat shouldBeIdentity = Rt * R;
    cv::Mat I = cv::Mat::eye(3, 3, shouldBeIdentity.type());

    return cv::norm(I, shouldBeIdentity) < 1e-6;
}

/**
 * @brief homo should be a 4x4 homogeneous matrix
 * 
 * @param homo 
 * @param R 
 * @param T 
 */
void Homo2RT(const cv::Mat& homo, cv::Mat& R, cv::Mat& T) {
    cv::Rect R_rect(0, 0, 3, 3);
    cv::Rect T_rect(3, 0, 1, 3);
    R = homo(R_rect);
    T = homo(T_rect);
}

void RT2Homo(const cv::Mat& R,const cv::Mat& T, cv::Mat& homo)
{
	cv::Mat HomoMtr;
	std::cout << R << std::endl;
	cv::Mat_<double> R1 = (cv::Mat_<double>(4, 3) << 
										R.at<double>(0, 0), R.at<double>(0, 1), R.at<double>(0, 2),
										R.at<double>(1, 0), R.at<double>(1, 1), R.at<double>(1, 2),
										R.at<double>(2, 0), R.at<double>(2, 1), R.at<double>(2, 2),
										0, 0, 0);
	std::cout << R1 << std::endl;
	cv::Mat_<double> T1 = (cv::Mat_<double>(4, 1) <<
										T.at<double>(0,0),
										T.at<double>(1,0),
										T.at<double>(2,0),
										1);
	cv::hconcat(R1, T1, HomoMtr);		//矩阵拼接, hconcat stands for horizontal concatentation
	homo = HomoMtr;
}



#endif