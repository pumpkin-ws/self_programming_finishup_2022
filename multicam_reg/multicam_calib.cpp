#include <condition_variable>
#include <mutex>
#include <thread>
#include <opencv2/opencv.hpp>
#include <termio.h>
#include <atomic>
#include "source/file_manip.hpp"
#include "source/realsense_driver.hpp"
#include "source/calib_intrinsics.hpp"

struct CurrentImg {
    std::mutex mtx;
    cv::Mat img;
} cur_img;

std::condition_variable cv_keypress;
std::mutex key_mtx;

std::atomic_int key{0};
std::atomic_bool key_set{false};
const std::string CAM1_SERIAL{"109622074093"};
const std::string CAM2_SERIAL{"108222250052"};

cv::Mat getCurrentImg();
int keyboardScan();
void streamImg(int cam_id);
void detectKey();
void utils();
void help();
std::string folder_name;

int main(int argc, char** argv) {

    /* create folder for storing images */
    help();
    printf("Type in folder name for image operation:(Press enter to confirm)\n");
    getline(std::cin, folder_name);
    int cam_id{1};
    printf("Type in camera id (1 - 3): (Press enter to confirm)\n");
    std::cin >> cam_id;
    if (cam_id > 3 || cam_id < 1) {
        printf("Invalid camera id, check label and retry\n");
        return 1;
    } 
    if (folder_name.size() != 0) {
        createDirectory(folder_name);
    } else {
        printf("Folder name invalid.\n");
        return 1;
    }
    
    std::thread detect_t(detectKey);
    std::thread util_t(utils);
    std::thread stream_t(streamImg, cam_id);

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

cv::Mat getCurrentImg() {
    std::lock_guard<std::mutex> lk(cur_img.mtx);
    return cur_img.img.clone();
}

void streamImg(int cam_id) {
    spark_cameras::SparkRealsense rs;
    std::string id;
    if (cam_id == 1) {
        id = CAM1_SERIAL;
    } 
    if (cam_id == 2) {
        id = CAM2_SERIAL;
    }
    rs.setParam(id, 1280, 720, 30);
    rs.startStream();
    printf("Camera stream started successfully.\n");
    cv::Mat frame;
    auto getFrame = [&frame, &rs](){
        std::lock_guard<std::mutex> lk (cur_img.mtx);
        rs.getRGBFrame(frame);
    };

    while(key.load(std::memory_order_seq_cst) != 27) {
        getFrame();
        cur_img.img = frame.clone();
        cv::imshow("frame", frame);
        cv::waitKey(50); 
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
            /* Store the current frame */
            case int('s'): {
                printf(" : Capture current image.\n");
                printf("Type in name of the image(e.g. fig0), and press enter.\n");
                cv::Mat img = getCurrentImg();
                cv::imshow(CURRENT_FRAME, img);
                std::string img_name;
                std::getline(std::cin, img_name);
                img_name = folder_name + "/" + img_name + ".jpg";
                printf("Image file name: %s\n", img_name.c_str());
                cv::imwrite(img_name, img);
                break;
            }
            /* Capture the current frame and display */
            case int('c'): {
                printf(" : Update current image.\n");
                cv::Mat img = getCurrentImg();
                cv::imshow("current frame", img);
                break;
            }
            /* Create new folder and switch to new folder */
            case int('f'): {
                printf(" : Enter and change folder name.\n");
                printf("Enter new folder name for calibration:\n");
                getline(std::cin, folder_name);
                createDirectory(folder_name);
                printf("Now operating in directory: %s\n", folder_name.c_str());
                break;
            }
            /* Calibrate camera intrinsics */
            case int('i'): {
                printf("Type in folder name for intrinsics calibration:\n");
                getline(std::cin, folder_name);
                printf("The directory has changed to: %s\n", folder_name.c_str());
                if (folder_name.size() == 0 || !boost::filesystem::exists(folder_name)) {
                    printf("Folder name is invalid or folder does not exists\n");
                    help();
                    break;
                }
                /* Perform calibration */
                std::vector<std::string> img_names = getAllFileName(folder_name, ".jpg");
                if (img_names.size() == 0) {
                    printf("No images ending in .jpg stored in folder %s.\n", folder_name.c_str());
                    break;
                }
                // sort the image names
                std::sort(img_names.begin(), img_names.end(), [](std::string name1, std::string name2)->bool {
                    int idx1 = atoi(name1.substr(0, name1.find_first_of('.')).c_str());
                    int idx2 = atoi(name2.substr(0, name2.find_first_of('.')).c_str());
                    return idx1 < idx2;
                });

                // load in the images
                std::vector<cv::Mat> input_imgs;
                for (int i = 0; i < img_names.size(); i++) {
                    std::string img_path = folder_name + "/" + img_names[i];
                    cv::Mat img = cv::imread(img_path);
                    input_imgs.push_back(img);
                }

                // find the image intrinsics
                cv::Mat intrinsics, distortion;
                std::vector<cv::Mat> rvecs, tvecs;
                calibrateIntrinsics(Pattern::CHESSBOARD, input_imgs, cv::Size(8, 5), 10, intrinsics, distortion, rvecs, tvecs);

                std::cout << "The camera intrinsics are: " << std::endl;
                std::cout << intrinsics << std::endl;

                std::cout << "The distortion coeffs are: " << std::endl;
                std::cout << distortion << std::endl; 

                std::string parameter_file = folder_name + "/image_parameter.yml";
                boost::filesystem::path p{parameter_file};
                if (!boost::filesystem::exists(p)) {
                    std::ofstream f;
                    f.open(parameter_file);
                    f.close();
                }
                cv::FileStorage fs(parameter_file, cv::FileStorage::WRITE);
                time_t raw_time;
                time(&raw_time);
                fs << "Date" << asctime(localtime(&raw_time));
                fs << "Type" << folder_name;
                fs << "Intrinsic" << intrinsics;
                fs << "Distortion" << distortion;
                fs << "ImageSize" << input_imgs[0].size();

                break;
            }
            case int('h'): {
                help();
                break;
            }
            /* calculate the object pose and save the pose */
            case int('o'): {
                // read in the camera intrinsics
                printf("Type in folder name for capturing object pose:\n");
                getline(std::cin, folder_name);    
                if (folder_name.empty()) {
                    printf("Folder name is empty. Press o and try again.\n");
                    break;
                }
                // folder_name = "./" + folder_name + "/";
                boost::filesystem::path p_folder{folder_name};
                if (!boost::filesystem::exists(p_folder)) {
                    printf("Folder name \"%s\" does not exist. Check folder name and retry by pressing o.\n", folder_name.c_str());
                    break;
                }
                printf("The directory has changed to: %s\n", folder_name.c_str());
                /* Get the image with common objects */
                std::string object_name = folder_name + "/object.jpg";
                cv::Mat object_img;
                object_img = cv::imread(object_name, cv::IMREAD_COLOR);

                // load in the image parameters
                std::string parameter_file = folder_name + "/image_parameter.yml";
                boost::filesystem::path p{parameter_file};
                if (!boost::filesystem::exists(p)) {
                    std::ofstream f;
                    f.open(parameter_file);
                    f.close();
                }
                cv::FileStorage fs(parameter_file, cv::FileStorage::READ);
                cv::Mat intrinsics, distortions;
                fs["Intrinsic"] >> intrinsics;
                fs["Distortion"] >> distortions;
                printf("The camera intrinsics are:\n");
                std::cout << intrinsics << std::endl;
                printf("The distortion coefficients are:\n");
                std::cout << distortions << std::endl;

                std::vector<cv::Point2f> tracked_centers;
                cv::Mat rvec, tvec;
                findAndDrawChessBoardCorners(object_img, tracked_centers, cv::Size(8, 5), false);
                findBoardPose(cv::Size(8, 5), 10, tracked_centers, intrinsics, distortions, rvec, tvec);
                cv::Mat rotation;
                cv::Rodrigues(rvec, rotation);
                cv::Mat homo;
                RT2Homo(rotation, tvec, homo);
                std::cout << "The object in camera homogeneous pose is: " << std::endl;
                std::cout << homo << std::endl;
                std::cout << "The camer in object homogeneous pose is: " << std::endl;
                std::cout << homo.inv() << std::endl;

                parameter_file = folder_name + "/CamInObject.yml";
                p = parameter_file;
                if (!boost::filesystem::exists(p)) {
                    std::ofstream f;
                    f.open(parameter_file);
                    f.close();
                }
                cv::FileStorage fsw(parameter_file, cv::FileStorage::WRITE);
                time_t raw_time;
                time(&raw_time);
                fsw << "Date" << asctime(localtime(&raw_time));
                fsw << "Type" << folder_name;
                fsw << "ObjectInCam" << homo;
                fsw << "CamInObject" << homo.inv();
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
    printf("******************HELP******************\n");
    printf("Functions for different keys are: \n");
    printf("s: store the current image\n");
    printf("c: capture and display the current image\n");
    printf("f: create new folder and switch to new folder\n");
    printf("h: print help info\n");
    printf("i: calibrate the intrinsics\n");
    printf("o: calculate object in pose\n");
    printf("******************End of Help******************\n");
}
