#ifndef CALIB_INTRINSICS_HPP_
#define CALIB_INTRINSICS_HPP_

#include "opencv2/opencv.hpp"
#include "matrix_util.hpp"

enum Pattern {CHESSBOARD, CIRCLES_GRID, ASYMMETRIC_CIRCLES_GRID};

int calibrateIntrinsics(
    Pattern p, 
    const std::vector<cv::Mat>& calib_imgs, 
    const cv::Size& pattern_size, 
    double pattern_dim, 
    cv::Mat& camera_mat, 
    cv::Mat& dist_coeff,
    std::vector<cv::Mat>& rvecs,
    std::vector<cv::Mat>& tvecs
);

/**
 * @brief findAndDrawSymmCircles - find and draw center points on symmetric circles
 * 
 * @param input 
 * @param tracked_centers output stores the tracked center calculation results
 * @param grid_size 
 * @param draw_result 
 * @return int 
 */
int findAndDrawSymmCircles(
    const cv::Mat& input, 
    std::vector<cv::Point2f>& tracked_centers, 
    const cv::Size& grid_size, 
    bool draw_result = true
);

/**
 * @brief find and draw chessboard corners 
 * 
 * @param input 
 * @param tracked_centers 
 * @param grid_size 
 * @param draw_result 
 * @return int 
 */
int findAndDrawChessBoardCorners(
    const cv::Mat& input, 
    std::vector<cv::Point2f>& tracked_centers, 
    const cv::Size& grid_size, 
    bool draw_result = true
);

/**
 * @brief Calculates one board pose based on the tracked centers
 * 
 * @param board_size 
 * @param square_size 
 * @param tracked_centers 
 * @param camera_matrix 
 * @param dist_coeffs 
 * @param rvec output the rotation vector of the board, need to be transformed to rotation matrix for hand eye calibration
 * @param tvec output the translation vector of the board
 * @return int 
 */
int findBoardPose(
    const cv::Size& board_size, 
    const float& pattern_dim, 
    const std::vector<cv::Point2f>& tracked_centers,
    const cv::Mat& camera_matrix,
    const cv::Mat& dist_coeffs,
    cv::Mat& rvector,
    cv::Mat& tvector
);

struct RobotPose{
    double x{0}, y{0}, z{0}, rx{0}, ry{0}, rz{0};
};


int calibrateHandInEye(
    const std::vector<RobotPose>& gripper_poses, // gripper to base pose inputs
    const std::vector<cv::Mat>& R_target, // rotation vector
    const std::vector<cv::Mat>& t_target2cam, // translation vector
    cv::Mat& R_camera2gripper,
    cv::Mat& t_camera2gripper
);

int saveIntrinsicsYAML(
    std::string file_path, 
    const cv::Mat& intrinsics, 
    const cv::Mat& distortion, 
    const cv::Size& image_size,
    std::string comments
);

/**
 * @brief calculate the eye in hand calibration matrix, the traslation units can be either meter or millimeter.
 * The rotation of the gripper will be rpy (in radians), and the rotation of target is rotation vector. The rotation vector
 * needs to be transformed into rotation matrix by using the Rodrigues forumla.
 * 
 * 
 * @param gripper2base the gripper to base transformation, 
 * @param target2camera 
 * @param R_cam2gripper 
 * @param t_cam2gripper 
 * @return int 
 */
int performEIHCalib (
    const std::vector<std::vector<double>>& gripper2base,
    const std::vector<std::vector<double>>& target2camera,
    std::string rotation_order,
    cv::Mat& R_cam2gripper,
    cv::Mat& t_cam2gripper
);

/**
 * @brief calculate the variation from mean value for each target pose
 * 
 * @param gripper2base the rotation of gripper pose is in rpy
 * @param target2camera the rotation of target pose is in rotation vector
 * @param R_camera2gripper the rotation part of camera to gripper
 * @param t_camera2gripper the translation part camera to gripper
 * @return std::vector<double> the distance from each target to base
 */
std::vector<double> EIHVerify(
    const std::vector<std::vector<double>>& gripper2base,
    const std::vector<std::vector<double>>& target2camera,
    const cv::Mat& R_camera2gripper,
    const cv::Mat& t_camera2gripper,
    std::string rotation_order
);

int performETHCalib(
    const std::vector<std::vector<double>>& gripper2base_set,
    const std::vector<std::vector<double>>& target2camera_set,
    std::string rotation_order,
    cv::Mat& R_cam2gripper,
    cv::Mat& t_cam2gripper
);

/**
 * @brief The relation between target and flange(or any tool) should remain constant.
 * 
 * @param gripper2base 
 * @param target2camera 
 * @param R_camera2gripper 
 * @param t_camera2gripper 
 * @return std::vector<double> the distances from each target to gripper
 */
std::vector<double> ETHVerify(
    const std::vector<std::vector<double>>& gripper2base,
    const std::vector<std::vector<double>>& target2camera,
    const cv::Mat& R_camera2gripper,
    const cv::Mat& t_camera2gripper,
    std::string rotation_order
);


#endif