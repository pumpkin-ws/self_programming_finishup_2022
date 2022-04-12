#include <opencv2/opencv.hpp>
#include "glog/logging.h"

void help() {
    printf("This program finds the camera intrincs of the three camers.\n");
    printf("To use this program, enter: \n");
    printf("calib_cam <path to cam folder 1> <path to cam folder 2> <path to cam folder 3>\n");
}

int main(int argc, char** argv) {
    /* Initialize glogging */
    /* Perform camera intrinsics calibration for the three cameras */
    if (argc != 3) {
        help();
        
    }
}