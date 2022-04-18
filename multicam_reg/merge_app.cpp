#include <condition_variable>
#include <mutex>
#include <thread>
#include <opencv2/opencv.hpp>
#include <termio.h>
#include <atomic>
#include "source/file_manip.hpp"
#include "source/realsense_driver.hpp"
#include "source/calib_intrinsics.hpp"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"

typedef pcl::PointCloud<pcl::PointXYZRGB> cloud;

struct CurrentImg {
    std::mutex mtx;
    cv::Mat img_cam1;
    cv::Mat img_cam2;
} cur_img;

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
    cloud_cam1 = combined_data.cloud_cam1;
    cloud_cam2 = combined_data.cloud_cam2;
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
    cloud::Ptr cloud_cam1(new cloud()), cloud_cam2(new cloud());


    auto getFrame = [&frame_cam1, &frame_cam2, &rs1, &rs2, &cloud_cam1, &cloud_cam2](){
        std::lock_guard<std::mutex> lk (combined_data.mtx);
        rs1.getData(frame_cam1, cloud_cam1);
        rs2.getData(frame_cam2, cloud_cam2);
        combined_data.img_cam1 = frame_cam1;
        combined_data.img_cam2 = frame_cam2;
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
    while(key.load(std::memory_order_seq_cst) != 27) {
        std::unique_lock<std::mutex> lk(key_mtx);
        cv_keypress.wait(lk, []{
            return key_set.load(std::memory_order_seq_cst) == true;
        });
        if (key.load(std::memory_order_seq_cst) == 27){
            printf("Breaking from utility thread\n");
            break;
        }
        const std::string CURRENT_FRAME{"current frame"};
        cv::namedWindow(CURRENT_FRAME);
        switch(key.load(std::memory_order_seq_cst)) {
            /* Merge view for a single frame */
            case int('s'): {
                std::string cam1_param = cam1_folder + "CamInObject.yml";
                std::string cam2_param = cam2_folder + "CamInObject.yml";
                cv::FileStorage fsw_cam1(cam1_param, cv::FileStorage::READ);
                cv::FileStorage fsw_cam2(cam2_param, cv::FileStorage::READ);
                cv::Mat d2c_cam1, d2c_cam2;
                fsw_cam1["D2C"] >> d2c_cam1;
                fsw_cam2["D2C"] >> d2c_cam2;
                printf("d2c cam1 \n");
                std::cout << d2c_cam1 << std::endl;
                printf("d2c cam2 \n");
                std::cout << d2c_cam2 << std::endl;
                break;
            }
            /* merge views in real time */
            case int('r'): {
                break;
            }
            case int('h'): {
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
