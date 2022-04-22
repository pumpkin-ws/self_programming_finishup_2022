#include "source/realsense_driver.hpp"
#include <chrono>

typedef pcl::PointCloud<pcl::PointXYZRGB> cloud;
const std::string CAM1_SERIAL{"109622074093"};
const std::string CAM2_SERIAL{"108222250052"};

int main(int argc, char** argv) {
    cv::Mat img;
    cloud::Ptr cloud_ptr(new cloud());

    spark_cameras::SparkRealsense rs1;
    rs1.setParam(CAM2_SERIAL, 1280, 720, 30);
    rs1.startStream();
    pcl::visualization::PCLVisualizer viewer("viewer");
    viewer.addPointCloud(cloud_ptr);

    int key;
    while(key != int('q')) {
        auto start = std::chrono::steady_clock::now();
        rs1.getData(img, cloud_ptr);
        auto end = std::chrono::steady_clock::now();
        std::chrono::duration<double, std::milli> dur = end - start;
        printf("Acquisition of point cloud took %.1f ms.\n", dur.count());
        cv::imshow("acquired image", img);
        key = cv::waitKey(20);
        viewer.updatePointCloud(cloud_ptr);
        viewer.spinOnce();                
    }

    return 0;
}
