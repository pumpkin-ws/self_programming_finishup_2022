#include <condition_variable>
#include <mutex>
#include <thread>
#include <opencv2/opencv.hpp>
#include <termio.h>
#include <atomic>
#include <future>

#include "source/file_manip.hpp"
#include "source/realsense_driver.hpp"
#include "source/calib_intrinsics.hpp"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl/visualization/pcl_visualizer.h"
#include "pcl/common/transforms.h"
#include "pcl/filters/passthrough.h"
#include "pcl/filters/statistical_outlier_removal.h"
#include "pcl/filters/radius_outlier_removal.h"
#include "pcl/filters/voxel_grid.h"
#include "pcl/io/ply_io.h"


typedef pcl::PointCloud<pcl::PointXYZRGB> cloud;

struct CombinedData {
    std::mutex mtx;
    cv::Mat img_cam1;
    cv::Mat img_cam2;
    cloud::Ptr cloud_cam1;
    cloud::Ptr cloud_cam2;

    CombinedData() {
        cloud_cam1 = cloud::Ptr(new cloud());
        cloud_cam2 = cloud::Ptr(new cloud());
    }    
} combined_data;

std::condition_variable cv_keypress;
std::mutex key_mtx;

std::atomic_int key{0};
std::atomic_bool key_set{false};
const std::string CAM1_SERIAL{"109622074093"};
const std::string CAM2_SERIAL{"108222250052"};

void getCurrent(cv::Mat&, cv::Mat&, cloud::Ptr, cloud::Ptr); 
int keyboardScan();
void streamImg();
void detectKey();
void utils();
void help();
std::string folder_name;
std::string cam1_folder, cam2_folder;


int main(int argc, char** argv) {

    /* create folder for storing images */
    printf("Type in folder name for cam1 pose info: (Press enter to confirm)\n");
    getline(std::cin, cam1_folder);
    printf("Type in folder name for cam2 pose info: (Press enter to confirm)\n");
    getline(std::cin, cam2_folder);
    
    std::thread detect_t(detectKey);
    std::thread util_t(utils);
    std::thread stream_t(streamImg);

    if (detect_t.joinable()) {
        detect_t.join();
    }
    if (util_t.joinable()) {
        util_t.join();
    }
    if (stream_t.joinable()) {
        stream_t.join();
    }
    return EXIT_SUCCESS;
}

void getCurrent(cv::Mat &frame_cam1, cv::Mat &frame_cam2, cloud::Ptr cloud_cam1, cloud::Ptr cloud_cam2) {
    std::lock_guard<std::mutex> lk(combined_data.mtx);
    frame_cam1 = combined_data.img_cam1.clone();
    frame_cam2 = combined_data.img_cam2.clone();

    cloud_cam1->points = combined_data.cloud_cam1->points;
    cloud_cam2->points = combined_data.cloud_cam2->points;
    return;
}

void streamImg() {
    spark_cameras::SparkRealsense rs1;
    rs1.setParam(CAM1_SERIAL, 1280, 720, 30);
    rs1.startStream();
    printf("Camera stream 1 started successfully.\n");

    spark_cameras::SparkRealsense rs2;
    rs2.setParam(CAM2_SERIAL, 1280, 720, 30);
    rs2.startStream();
    printf("Camera stream 2 started successfully.\n");
    
    cv::Mat frame_cam1, frame_cam2;

    auto getFrame = [&frame_cam1, &frame_cam2, &rs1, &rs2](){
        std::lock_guard<std::mutex> lk (combined_data.mtx);
        cloud::Ptr cloud_cam1(new cloud());
        cloud::Ptr cloud_cam2(new cloud());
        auto get_cloud1 = [&rs1, &cloud_cam1, &frame_cam1](){
            rs1.getData(frame_cam1, cloud_cam1);
        };
        auto get_cloud2 = [&rs2, &cloud_cam2, &frame_cam2](){
            rs2.getData(frame_cam2, cloud_cam2);
        };
        std::thread t_get_cloud1(get_cloud1);
        std::thread t_get_cloud2(get_cloud2);
        if (t_get_cloud1.joinable()) {
            t_get_cloud1.join();
        }
        if (t_get_cloud2.joinable()) {
            t_get_cloud2.join();
        }
        combined_data.img_cam1 = frame_cam1.clone();
        combined_data.img_cam2 = frame_cam2.clone();
        combined_data.cloud_cam1 = cloud_cam1;
        combined_data.cloud_cam2 = cloud_cam2;
    };
    std::string cam1_window_name = "cam1:" + CAM1_SERIAL;
    std::string cam2_window_name = "cam2:" + CAM2_SERIAL;
    cv::namedWindow(cam1_window_name);
    cv::namedWindow(cam2_window_name);

    while(key.load(std::memory_order_seq_cst) != 27) {
        getFrame();        
        cv::imshow(cam1_window_name, frame_cam1);
        cv::imshow(cam2_window_name, frame_cam2);
        cv::waitKey(10); 
    }

    std::cout << "Breaking from image stream thread" << std::endl;
}

void detectKey() {
    while(key.load(std::memory_order_seq_cst) != 27) {
        std::unique_lock<std::mutex> lk(key_mtx);
        cv_keypress.wait(lk, []{
            return key_set.load(std::memory_order_seq_cst) == false;
        });
        key.store(keyboardScan(), std::memory_order_seq_cst);
        key_set.store(true, std::memory_order_seq_cst);
        cv_keypress.notify_one();
    }
    printf("Breaking from key detection thread.\n");
}

void utils() {
    pcl::visualization::PCLVisualizer viewer("viewer");

    while(key.load(std::memory_order_seq_cst) != 27) {
        std::unique_lock<std::mutex> lk(key_mtx);
        cv_keypress.wait(lk, []{
            return key_set.load(std::memory_order_seq_cst) == true;
        });
        if (key.load(std::memory_order_seq_cst) == 27){
            printf("Breaking from utility thread\n");
            break;
        }
        // const std::string CURRENT_FRAME{"current frame"};
        // cv::namedWindow(CURRENT_FRAME);
        switch(key.load(std::memory_order_seq_cst)) {
            /* Merge view for a single frame */
            case int('s'): {
                printf(": Merge point clouds from a single frame.\n");
                printf("Press q to exit.\n");
                std::string cam1_param = cam1_folder + "/CamInObject.yml";
                std::string cam2_param = cam2_folder + "/CamInObject.yml";
                std::string cam1_d2c = cam1_folder + "/d2c.yml";
                std::string cam2_d2c = cam2_folder + "/d2c.yml";

                cv::FileStorage fsw_cam1(cam1_param, cv::FileStorage::READ);
                cv::FileStorage fsw_cam2(cam2_param, cv::FileStorage::READ);
                cv::FileStorage fs_cam1_d2c(cam1_d2c, cv::FileStorage::READ);
                cv::FileStorage fs_cam2_d2c(cam2_d2c, cv::FileStorage::READ);
                std::cout << std::boolalpha << fsw_cam1.isOpened() << std::endl;
                std::cout << std::boolalpha << fsw_cam2.isOpened() << std::endl;
                cv::Mat d2c_cam1, d2c_cam2, objInCam1, objInCam2, cam1InObj, cam2InObj;
                fs_cam1_d2c["D2C"] >> d2c_cam1;
                fs_cam2_d2c["D2C"] >> d2c_cam2;
                fsw_cam1["ObjectInCam"] >> objInCam1;
                fsw_cam2["ObjectInCam"] >> objInCam2;
                fsw_cam1["CamInObject"] >> cam1InObj;
                fsw_cam2["CamInObject"] >> cam2InObj;

                cloud::Ptr cloud_cam1 (new cloud()), cloud_cam2 (new cloud());
                cv::Mat img_cam1, img_cam2;

                getCurrent(img_cam1, img_cam2, cloud_cam1, cloud_cam2);

                
                pcl::PassThrough<pcl::PointXYZRGB> ps;
                
                ps.setFilterFieldName("z");
                ps.setFilterLimits(0, 2);
                
                auto ps_start = std::chrono::steady_clock::now();
                ps.setInputCloud(cloud_cam1);
                ps.filter(*cloud_cam1);
                ps.setInputCloud(cloud_cam2);
                ps.filter(*cloud_cam2);
                auto ps_end = std::chrono::steady_clock::now();
                std::chrono::duration<double, std::milli> dur = ps_end - ps_start;
                printf("Time took for pass through filter is %.1f.\n", dur.count());

                
                pcl::VoxelGrid<pcl::PointXYZRGB> vg;
                float leaf_size{0.003};
                vg.setLeafSize(leaf_size, leaf_size, leaf_size);

                auto vg_start = std::chrono::steady_clock::now();
                vg.setInputCloud(cloud_cam1);
                vg.filter(*cloud_cam1);
                vg.setInputCloud(cloud_cam2);
                vg.filter(*cloud_cam2);
                auto vg_end = std::chrono::steady_clock::now();
                dur = vg_end - vg_start;
                printf("Time took for voxel grid filter is %.1f.\n", dur.count());

                pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
                sor.setInputCloud(cloud_cam1);
                sor.setMeanK(50);
                sor.setStddevMulThresh(0.5);
                sor.filter(*cloud_cam1);
                
                sor.setInputCloud(cloud_cam2);
                sor.filter(*cloud_cam2);

                // pcl::RadiusOutlierRemoval<pcl::PointXYZRGB> ror;
                // ror.setMinNeighborsInRadius(10);
                // ror.setRadiusSearch(0.005);
                // ror.setInputCloud(cloud_cam1);
                // ror.filter(*cloud_cam1);

                // ror.setInputCloud(cloud_cam2);
                // ror.filter(*cloud_cam2);

                auto homoMat2Eigen = [](const cv::Mat& input, Eigen::Matrix4d& output, bool is_mm) {
                    for (int i = 0; i < 4; i++) {
                        for (int j = 0; j < 4; j++) {
                            if (j == 3 && i != 3) {
                                if (is_mm == true) {
                                    output.row(i)(j) = input.at<double>(i, j) / 1000.0;
                                } else {
                                    output.row(i)(j) = input.at<double>(i, j);
                                }
                            } else {
                                output.row(i)(j) = input.at<double>(i, j);
                            }
                        }
                    }
                };

                Eigen::Matrix4d T_d2c_cam1, T_c2d_cam1, T_d2c_cam2, T_cam1_obj, T_obj_cam2;
                homoMat2Eigen(d2c_cam1, T_d2c_cam1, false);
                homoMat2Eigen(d2c_cam2, T_d2c_cam2, false);
                homoMat2Eigen(cam1InObj, T_cam1_obj, true);
                homoMat2Eigen(objInCam2, T_obj_cam2, true);
                T_c2d_cam1 = T_d2c_cam1.inverse();

                std::cout << "object in cam1 is " << std::endl;
                std::cout << T_cam1_obj << std::endl;
                std::cout << "cam2 in object is " << std::endl;
                std::cout << T_obj_cam2 << std::endl;
                std::cout << "cam2 depth to color is" << std::endl;
                std::cout << T_d2c_cam2 << std::endl;
                std::cout << "cam0 color to depth is" << std::endl;
                std::cout << T_c2d_cam1 << std::endl;
                

                /* Transformation from depth camera 2 to depth camera 1 */
                Eigen::Matrix4d T_dcam2_dcam1 = T_c2d_cam1 * T_cam1_obj.inverse() * T_obj_cam2.inverse() * T_d2c_cam2;
                // Eigen::Matrix4d T_dcam2_dcam1 = T_cam1_obj.inverse() * T_obj_cam2.inverse();

                cloud::Ptr output_cam2 (new cloud());
                auto transform_start = std::chrono::steady_clock::now();
                pcl::transformPointCloud(*cloud_cam2, *output_cam2, T_dcam2_dcam1.cast<float>());
                auto transform_end = std::chrono::steady_clock::now();
                dur = transform_end - transform_start;
                printf("Point cloud transformation took %.1fms.\n", dur.count());

                pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> cloud1_color(cloud_cam1, 255, 0, 0); // RED
                pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> cloud2_color(cloud_cam2, 0, 0, 255); // BLUE
                pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> output_color(output_cam2, 0, 255, 0); // GREEN
                viewer.addPointCloud(cloud_cam1, cloud1_color, "cam1");
                viewer.addPointCloud(cloud_cam2, cloud2_color, "cam2");
                viewer.addPointCloud(output_cam2, output_color, "cam2_output");

                lk.unlock();
                printf("Display point cloud. Press \"r\" to save point clouds\n");
                while(key.load(std::memory_order_seq_cst) != int('q') && key.load(std::memory_order_seq_cst) != 27) {
                    viewer.spinOnce();
                    key_set.store(false, std::memory_order_seq_cst);
                    cv_keypress.notify_one();
                    /* Save the current cloud */
                    if (key.load(std::memory_order_seq_cst) == int('r')) {
                        printf(" : Saving current cloud and exit.\n");
                        cloud::Ptr save_cloud(new cloud());
                        *save_cloud = *cloud_cam1 + *output_cam2;
                        pcl::io::savePCDFile("merge.pcd", *save_cloud);
                        pcl::io::savePCDFile("view1.pcd", *cloud_cam1);
                        pcl::io::savePCDFile("view2.pcd", *cloud_cam2);
                        pcl::io::savePCDFile("view2_transformed.pcd", *output_cam2);

                        pcl::io::savePLYFile("merge.ply", *save_cloud);
                        pcl::io::savePLYFile("view1.ply", *cloud_cam1);
                        pcl::io::savePLYFile("view2.ply", *cloud_cam2);
                        pcl::io::savePLYFile("view2_transformed.ply", *output_cam2);
                        
                        break;
                    }
                }
                viewer.removeAllPointClouds();
                printf(": Exiting from display loop.\n");
                break;
            }
            /* merge views in real time */
            case int('r'): {
                printf(" : Merging views in real time.\n");
                printf("Press q to exit.\n");
                std::string cam1_param = cam1_folder + "/CamInObject.yml";
                std::string cam2_param = cam2_folder + "/CamInObject.yml";
                std::string cam1_d2c = cam1_folder + "/d2c.yml";
                std::string cam2_d2c = cam2_folder + "/d2c.yml";

                cv::FileStorage fsw_cam1(cam1_param, cv::FileStorage::READ);
                cv::FileStorage fsw_cam2(cam2_param, cv::FileStorage::READ);
                cv::FileStorage fs_cam1_d2c(cam1_d2c, cv::FileStorage::READ);
                cv::FileStorage fs_cam2_d2c(cam2_d2c, cv::FileStorage::READ);

                cv::Mat d2c_cam1, d2c_cam2, objInCam1, objInCam2, cam1InObj, cam2InObj;
                fs_cam1_d2c["D2C"] >> d2c_cam1;
                fs_cam2_d2c["D2C"] >> d2c_cam2;
                fsw_cam1["ObjectInCam"] >> objInCam1;
                fsw_cam2["ObjectInCam"] >> objInCam2;
                fsw_cam1["CamInObject"] >> cam1InObj;
                fsw_cam2["CamInObject"] >> cam2InObj;

                auto homoMat2Eigen = [](const cv::Mat& input, Eigen::Matrix4d& output) {
                    for (int i = 0; i < 4; i++) {
                        for (int j = 0; j < 4; j++) {
                            output.row(i)(j) = input.at<double>(i, j);
                        }
                    }
                };

                Eigen::Matrix4d T_d2c_cam1, T_c2d_cam1, T_d2c_cam2, T_cam1_obj, T_obj_cam2;
                homoMat2Eigen(d2c_cam1, T_d2c_cam1);
                homoMat2Eigen(d2c_cam2, T_d2c_cam2);
                homoMat2Eigen(cam1InObj, T_cam1_obj);
                homoMat2Eigen(objInCam2, T_obj_cam2);

                T_c2d_cam1 = T_d2c_cam1.inverse();

                /* Transformation from depth camera 2 to depth camera 1 */
                Eigen::Matrix4d T_dcam2_dcam1 = T_d2c_cam1 * T_cam1_obj.inverse() * T_obj_cam2.inverse() * T_d2c_cam2;
                
                cloud::Ptr cloud_cam1 (new cloud()), cloud_cam2 (new cloud()), output_cam2 (new cloud());
                pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> cloud1_color(cloud_cam1, 255, 0, 0);
                pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> cloud2_color(cloud_cam2, 0, 0, 255);
                pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> output_color(output_cam2, 0, 255, 0);
                cv::Mat img_cam1, img_cam2;
                viewer.addPointCloud(cloud_cam1, cloud1_color, "cam1");
                viewer.addPointCloud(cloud_cam2, cloud2_color, "cam2");
                viewer.addPointCloud(output_cam2, output_color, "cam2_output");

                printf("Entering display loop\n");
                lk.unlock();
                while(key.load(std::memory_order_seq_cst) != int('q')) {
                    getCurrent(img_cam1, img_cam2, cloud_cam1, cloud_cam2);
                    pcl::transformPointCloud(*cloud_cam2, *output_cam2, T_dcam2_dcam1.cast<float>());
                    viewer.updatePointCloud(cloud_cam1, cloud1_color, "cam1");
                    viewer.updatePointCloud(cloud_cam2, cloud2_color, "cam2");
                    viewer.updatePointCloud(cloud_cam1, cloud1_color, "cam1");
                    viewer.spinOnce();
                    key_set.store(false, std::memory_order_seq_cst);
                    cv_keypress.notify_one();
                }
                printf(": Exiting from display loop.\n");
                break;
            }
            case int('h'): {
                printf(" : help is pressed.\n");
                help();
                break;
            }
            default: {
                printf(" : The key pressed is %d\n", uchar(key.load(std::memory_order_seq_cst)));
                help();
                break;
            }
        } 
        key_set.store(false, std::memory_order_seq_cst);
        cv_keypress.notify_one();
    }
}

int keyboardScan() {
    int in;
    struct termios new_settings;
    struct termios stored_settings;
    tcgetattr(0, &stored_settings);
    new_settings = stored_settings;
    new_settings.c_lflag &= (~ICANON);
    new_settings.c_cc[VTIME] = 0;
    tcgetattr(0, &stored_settings);
    new_settings.c_cc[VMIN] = 1;
    tcsetattr(0, TCSANOW, &new_settings);
    in = getchar();
    tcsetattr(0, TCSANOW, &stored_settings);
    return in;
}

void help() {
    printf("Functions for different keys are: \n");
    printf("s: merge views from a single frame\n");
    printf("r: merge views in realtime\n");
}
