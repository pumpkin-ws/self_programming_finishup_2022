#include "calib_intrinsics.hpp"

int findAndDrawSymmCircles(
    const cv::Mat& input, 
    std::vector<cv::Point2f>& tracked_centers, 
    const cv::Size& grid_size, 
    bool draw_result) {

    cv::Mat track_result;
    cv::Mat display_mat = input.clone();
    tracked_centers.clear();
    if(cv::findCirclesGrid(input, grid_size, track_result)) {
        // if track is successful
        for (int i = 0; i < track_result.rows; i++) {
            tracked_centers.push_back(track_result.at<cv::Point2f>(0, i));
            if (draw_result == true) {
                cv::circle(display_mat, tracked_centers[i], 10, cv::Scalar(0, 0, 255), 5, cv::FILLED);
                cv::putText(display_mat, std::to_string(i), tracked_centers[i], cv::FONT_HERSHEY_COMPLEX, 1, cv::Scalar(0, 255, 255));
            }
        }
        if (draw_result == true) {
            cv::imshow("found circles", display_mat);
            cv::waitKey(0);
        }
        return 0;
    } else {
        // if find circle grid is unsuccessful, return unsuccessful code of 1
        return 1;   
    };

};

int findAndDrawChessBoardCorners (
    const cv::Mat& input, 
    std::vector<cv::Point2f>& tracked_centers, 
    const cv::Size& grid_size, 
    bool draw_result) 
{
    cv::Mat track_result;
    cv::Mat display_mat = input.clone();
    tracked_centers.clear();
    bool pattern_found = cv::findChessboardCorners(input, grid_size, tracked_centers);
    if (!pattern_found) {
        // if track is unsuccessful
        return 1;
    } else {
        // refine the chessboard corners if the track is successful
        cv::Mat gray;
        cv::cvtColor(input, gray, cv::COLOR_BGR2GRAY);
        cv::cornerSubPix(
            gray, 
            tracked_centers, 
            cv::Size(11, 11), 
            cv::Size(-1, -1), 
            cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.01)
        );
        // if track is successful, save the result to the return vector
        for (int i = 0; i < track_result.rows; i++) {
            tracked_centers.push_back(track_result.at<cv::Point2f>(0, i));
        }
        if (draw_result == true) {
            cv::drawChessboardCorners(display_mat, grid_size, tracked_centers, pattern_found);
            cv::imshow("Found corners", display_mat);
            cv::waitKey(0);
        }
        return 0;
    };

};

int calibrateIntrinsics(
    Pattern p, 
    const std::vector<cv::Mat>& calib_imgs, 
    const cv::Size& pattern_size, 
    double pattern_dim, 
    cv::Mat& camera_mat, 
    cv::Mat& distortion,
    std::vector<cv::Mat>& rvecs_out,
    std::vector<cv::Mat>& tvecs_out) 
{
    switch (p) {
        case Pattern::CIRCLES_GRID: {
            /* Find the corner lists */
            std::vector<std::vector<cv::Point2f>> corner_lists;
            std::vector<cv::Point2f> corner_points;
            for (int i{0}; i < calib_imgs.size(); i++) {
                if(findAndDrawSymmCircles(calib_imgs[i], corner_points, pattern_size, false) != 0) {
                    // TODO: Can be glog here to log the error information
                    std::cerr << "Unable to find corner points" << std::endl;
                    return 1;
                };
                corner_lists.push_back(corner_points);
            }

            /* Generate the object points based on the grid size and the grid dimensions */
            std::vector<std::vector<cv::Point3f>> object_lists;
            std::vector<cv::Point3f> object_points;
            for (int i = 0; i < pattern_size.height; i++) {
                for (int j = 0; j < pattern_size.width; j++) {
                    object_points.emplace_back(cv::Point3f(j*pattern_dim, i*pattern_dim, 0));
                }
            }
            object_lists.resize(corner_lists.size(), object_points);

            /* Define output parameters
                1 - camera_matrix: the intrinsic matrix of the camera
                2 - dist_coeffs: the five parameter distortion coefficients
                3 - rvecs, tvec are the rotational and translational parameters of the calibration boards
             */
            cv::Mat camera_matrix = cv::Mat::eye(3, 3, CV_64F);
            cv::Mat dist_coeff = cv::Mat::zeros(5, 1, CV_64F);
            cv::Size image_size = calib_imgs[0].size();
            std::vector<cv::Mat> rvecs, tvecs;

            double rms = cv::calibrateCamera(object_lists, corner_lists, image_size, camera_matrix, dist_coeff, rvecs, tvecs);

            camera_mat = camera_matrix.clone();
            distortion = dist_coeff.clone();
            rvecs_out.clear();
            tvecs_out.clear();
            rvecs_out = rvecs;
            tvecs_out = tvecs;
            
            break;
        }
        case Pattern::CHESSBOARD: {
            std::vector<std::vector<cv::Point2f>> corner_lists;
            std::vector<cv::Point2f> corner_points;
            for (int i = 0; i < calib_imgs.size(); i++) {
                if (findAndDrawChessBoardCorners(calib_imgs[i], corner_points, pattern_size, false) != 0) {
                    std::cerr << "Unable to find corner points" << std::endl;
                    return 1;
                } else {
                    corner_lists.push_back(corner_points);
                }
            }
            /* Generate the object points based on the grid size and the grid dimensions */
            std::vector<std::vector<cv::Point3f>> object_lists;
            std::vector<cv::Point3f> object_points;
            for (int i = 0; i < pattern_size.height; i++) {
                for (int j = 0; j < pattern_size.width; j++) {
                    object_points.emplace_back(cv::Point3f(i*pattern_dim, j*pattern_dim, 0));
                }
            }
            object_lists.resize(corner_lists.size(), object_points);
            /* Define output parameters
                1 - camera_matrix: the intrinsic matrix of the camera
                2 - dist_coeffs: the five parameter distortion coefficients
                3 - rvecs, tvec are the rotational and translational parameters of the calibration boards
             */
            cv::Mat camera_matrix = cv::Mat::eye(3, 3, CV_64F);
            cv::Mat dist_coeff = cv::Mat::zeros(5, 1, CV_64F);
            cv::Size image_size = calib_imgs[0].size();
            std::vector<cv::Mat> rvecs, tvecs;
            
            double rms = cv::calibrateCamera(object_lists, corner_lists, image_size, camera_matrix, dist_coeff, rvecs, tvecs);
            camera_mat = camera_matrix.clone();
            distortion = dist_coeff.clone();
            rvecs_out.clear();
            tvecs_out.clear();
            rvecs_out = rvecs;
            tvecs_out = tvecs;
            break;
        }
        case Pattern::ASYMMETRIC_CIRCLES_GRID: {
            
            break;
        }
    }
    return 0;
}

/**
 * @brief find the board pose given the 
 * 
 * @param board_size 
 * @param pattern_dim 
 * @param tracked_centers 
 * @param camera_matrix camera intrinsic matrix
 * @param dist_coeffs distortion coefficients 
 * @param rvec output
 * @param tvec output
 * @return int 
 */
int findBoardPose(
    const cv::Size& board_size, 
    const float& pattern_dim, 
    const std::vector<cv::Point2f>& tracked_centers,
    const cv::Mat& camera_matrix,
    const cv::Mat& dist_coeffs,
    cv::Mat& rvec,
    cv::Mat& tvec
)
{
    /* Generate the object points based on the grid size and the grid dimensions */
    std::vector<cv::Point3f> object_points;
    for (int i = 0; i < board_size.height; i++) {
        for (int j = 0; j < board_size.width; j++) {
            object_points.emplace_back(cv::Point3f(i*pattern_dim, j*pattern_dim, 0));
        }
    }

    /* Calculate translation and rotation with solvePnP */
    cv::Mat rvector = cv::Mat::zeros(3, 1, CV_64F);
    cv::Mat tvector = cv::Mat::zeros(3, 1, CV_64F);
    cv::Mat tracked_centers_map = cv::Mat(1, tracked_centers.size(), CV_32FC2);
    for (int i = 0; i < tracked_centers.size(); i++) {
        tracked_centers_map.at<cv::Point2f>(0, i) = tracked_centers[i];
    }
    if (cv::solvePnP(object_points, tracked_centers_map, camera_matrix, dist_coeffs, rvector, tvector) == false) {
        return 1;
    };
    rvec = rvector.clone();
    tvec = tvector.clone();
    return 0;
};

int calibrateHandInEye(
    const std::vector<RobotPose>& gripper_poses, // gripper to base pose inputs
    const std::vector<cv::Mat>& R_target, // rotation vector - will be converted to rotation matrix later
    const std::vector<cv::Mat>& t_target2cam, // translation vector
    cv::Mat& R_camera2gripper,
    cv::Mat& t_camera2gripper
) 
{
    /* The primary purpose of packaging another function is to convert the poses in natural expression into consistent format
        for calibrateHandEye to process easier.
     */
    if (gripper_poses.size() !=  R_target.size()) {
        fprintf(stderr, "The number of robot poses is inconsistent with the number of targets.\n");
        return 1;
    } 
    /* Convert gripper pose into rotation matrix */
    std::vector<cv::Mat> R_gripper2base;
    std::vector<cv::Mat> t_gripper2base;
    
    for (int i{0}; i < gripper_poses.size(); i++) {
        cv::Vec3d euler_angles(gripper_poses[i].rx, gripper_poses[i].ry, gripper_poses[i].rz);
        cv::Mat rot = eulerAnglesToRotationMatrix(euler_angles);
        cv::Mat trans = (cv::Mat_<double>(3, 1) << gripper_poses[i].x, gripper_poses[i].y, gripper_poses[i].z);
        R_gripper2base.push_back(rot);
        t_gripper2base.push_back(trans);
    }

    /* Convert the target in camera to rotation matrix */
    std::vector<cv::Mat> R_target2cam;

    for (int i{0}; i < R_target.size(); i++) {
        cv::Mat rot;
        cv::Rodrigues(R_target[i], rot);
        R_target2cam.push_back(rot);
    }

    cv::calibrateHandEye(
        R_gripper2base,
        t_gripper2base,
        R_target2cam,
        t_target2cam,
        R_camera2gripper,
        t_camera2gripper
    );

    return 1;
};

int performEIHCalib (
    const std::vector<std::vector<double>>& gripper2base_set,
    const std::vector<std::vector<double>>& target2camera_set,
    std::string rotation_order,
    cv::Mat& R_cam2gripper,
    cv::Mat& t_cam2gripper
) {
    std::vector<cv::Mat> gripper2base_rot;
    std::vector<cv::Mat> gripper2base_trans;
    std::vector<cv::Mat> target2camera_rot;
    std::vector<cv::Mat> target2camera_trans;

    for (int i = 0; i < gripper2base_set.size(); i++) {
        // get the rotation matrix and the translation matrix
        cv::Vec3d rot(
            gripper2base_set[i][3],
            gripper2base_set[i][4],
            gripper2base_set[i][5]);
        cv::Mat rot_mat = eulerAnglesToRotationMatrix(rot, rotation_order); 
        gripper2base_rot.push_back(rot_mat);

        cv::Mat trans_mat = (cv::Mat_<double>(3, 1) << gripper2base_set[i][0], gripper2base_set[i][1], gripper2base_set[i][2]);
        gripper2base_trans.push_back(trans_mat);
    }

    for (int i = 0; i < target2camera_set.size(); i++) {
        // get the rotation matrix
        cv::Vec3f rot(
            target2camera_set[i][3],
            target2camera_set[i][4],
            target2camera_set[i][5]
        );
        cv::Mat rot_mat;
        cv::Rodrigues(rot, rot_mat);
        cv::Mat trans_mat = (cv::Mat_<double>(3, 1) << target2camera_set[i][0], target2camera_set[i][1], target2camera_set[i][2]);
        target2camera_rot.push_back(rot_mat);
        target2camera_trans.push_back(trans_mat);
    }
    cv::calibrateHandEye(
        gripper2base_rot, 
        gripper2base_trans, 
        target2camera_rot, 
        target2camera_trans, 
        R_cam2gripper, 
        t_cam2gripper, 
        cv::CALIB_HAND_EYE_TSAI
    );
    
    return 0;
}

std::vector<double> EIHVerify(
    const std::vector<std::vector<double>>& gripper2base,
    const std::vector<std::vector<double>>& target2camera,
    const cv::Mat& R_camera2gripper,
    const cv::Mat& t_camera2gripper,
    std::string rotation_order
) {
    /* Transform the gripper to base into homogeneous matrix */
    std::vector<cv::Mat> homo_gripper2base;
    std::vector<cv::Mat> homo_target2camera;
    cv::Mat homo_camera2gripper;

    for (int i = 0; i < gripper2base.size(); i++) {
        cv::Mat trans_mat = (cv::Mat_<double>(3, 1) << gripper2base[i][0], gripper2base[i][1], gripper2base[i][2]);
        cv::Vec3d rot (gripper2base[i][3], gripper2base[i][4], gripper2base[i][5]);
        cv::Mat rot_mat = eulerAnglesToRotationMatrix(rot, rotation_order);
        cv::Mat homo;
        RT2Homo(rot_mat, trans_mat, homo);
        homo_gripper2base.push_back(homo);
    }

    for (int i = 0; i < target2camera.size(); i++) {
        cv::Mat trans_mat = (cv::Mat_<double>(3, 1) << target2camera[i][0], target2camera[i][1], target2camera[i][2]);
        cv::Vec3d rot (target2camera[i][3], target2camera[i][4], target2camera[i][5]);
        cv::Mat rot_mat = cv::Mat(3, 3, CV_64FC1, cv::Scalar::all(0));
        cv::Rodrigues(rot, rot_mat);
        cv::Mat homo;
        RT2Homo(rot_mat, trans_mat, homo);
        homo_target2camera.push_back(homo);
    }
    /* 
        calculate the target pose in robot world coordinate 
    */
    RT2Homo(R_camera2gripper, t_camera2gripper, homo_camera2gripper);
    for (int i = 0; i < homo_gripper2base.size(); i++) {
        cv::Mat pose = homo_gripper2base[i] * homo_camera2gripper * homo_target2camera[i];
    }

    return std::vector<double>();
};


int performETHCalib(
    const std::vector<std::vector<double>>& gripper2base_set,
    const std::vector<std::vector<double>>& target2camera_set,
    std::string rotation_order,
    cv::Mat& R_cam2gripper,
    cv::Mat& t_cam2gripper
) {
    std::vector<cv::Mat> base2gripper_rot;
    std::vector<cv::Mat> base2gripper_trans;
    std::vector<cv::Mat> target2camera_rot;
    std::vector<cv::Mat> target2camera_trans;

    for (int i = 0; i < gripper2base_set.size(); i++) {
        // get the rotation matrix and the translation matrix
        cv::Vec3d rot(
            gripper2base_set[i][3],
            gripper2base_set[i][4],
            gripper2base_set[i][5]);
        cv::Mat rot_mat = eulerAnglesToRotationMatrix(rot, rotation_order); 
        base2gripper_rot.push_back(rot_mat.t());

        cv::Mat trans_mat = (cv::Mat_<double>(3, 1) << gripper2base_set[i][0], gripper2base_set[i][1], gripper2base_set[i][2]);
        base2gripper_trans.push_back(-rot_mat.t() * trans_mat);
        std::cout << "The translation of base to gripper " << i << std::endl;
        std::cout << -rot_mat.t() * trans_mat << std::endl;
    }


    for (int i = 0; i < target2camera_set.size(); i++) {
        // get the rotation matrix
        cv::Vec3f rot(
            target2camera_set[i][3],
            target2camera_set[i][4],
            target2camera_set[i][5]
        );
        cv::Mat rot_mat;
        cv::Rodrigues(rot, rot_mat);
        cv::Mat trans_mat = (cv::Mat_<double>(3, 1) << target2camera_set[i][0], target2camera_set[i][1], target2camera_set[i][2]);
        target2camera_rot.push_back(rot_mat);
        target2camera_trans.push_back(trans_mat);
    }

    cv::calibrateHandEye(
        base2gripper_rot, 
        base2gripper_trans, 
        target2camera_rot, 
        target2camera_trans, 
        R_cam2gripper, 
        t_cam2gripper, 
        cv::CALIB_HAND_EYE_TSAI
    );
    
    return 0;
}

std::vector<double> ETHVerify(
    const std::vector<std::vector<double>>& gripper2base,
    const std::vector<std::vector<double>>& target2camera,
    const cv::Mat& R_camera2gripper,
    const cv::Mat& t_camera2gripper,
    std::string rotation_order
) {
    /* Transform the gripper to base into homogeneous matrix */
    std::vector<cv::Mat> homo_target2camera;
    std::vector<cv::Mat> homo_base2gripper;
    cv::Mat homo_camera2gripper;

    for (int i = 0; i < gripper2base.size(); i++) {
        // get the rotation matrix and the translation matrix
        cv::Vec3d rot(
            gripper2base[i][3],
            gripper2base[i][4],
            gripper2base[i][5]);
        cv::Mat rot_mat = eulerAnglesToRotationMatrix(rot, rotation_order); 
        cv::Mat trans_mat = -rot_mat.t() *(cv::Mat_<double>(3, 1) << gripper2base[i][0], gripper2base[i][1], gripper2base[i][2]);
        cv::Mat homo;
        RT2Homo(rot_mat.t(), trans_mat, homo);
        homo_base2gripper.push_back(homo);
    }

    RT2Homo(R_camera2gripper, t_camera2gripper, homo_camera2gripper
    );

    /* Transform from trans + rot vec to homo matrix */
    for (int i = 0; i < target2camera.size(); i++) {
        cv::Mat trans_mat = (cv::Mat_<double>(3, 1) << target2camera[i][0], target2camera[i][1], target2camera[i][2]);
        cv::Vec3d rot (target2camera[i][3], target2camera[i][4], target2camera[i][5]);
        cv::Mat rot_mat = cv::Mat(3, 3, CV_64FC1, cv::Scalar::all(0));
        cv::Rodrigues(rot, rot_mat);
        cv::Mat homo;
        RT2Homo(rot_mat, trans_mat, homo);
        homo_target2camera.push_back(homo);
    }

    for (int i = 0; i < target2camera.size(); i++) {
        printf("-------------%d------------\n", i);
        cv::Mat pose = homo_base2gripper[i] * homo_camera2gripper * homo_target2camera[i];
    }

    return std::vector<double>();

};

