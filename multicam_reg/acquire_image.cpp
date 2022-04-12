#include <condition_variable>
#include <mutex>
#include <thread>
#include <opencv2/opencv.hpp>
#include <termio.h>
#include <atomic>
#include "source/realsense_driver.hpp"

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

struct CurrentImg {
    std::mutex mtx;
    cv::Mat img;
} cur_img;

std::condition_variable cv_keypress;
std::mutex key_mtx;

std::atomic_int key{0};
std::atomic_bool key_set{false};

cv::Mat getCurrentImg();

void streamImg();

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

void storeImage() {
    while(key.load(std::memory_order_seq_cst) != 27) {
        std::unique_lock<std::mutex> lk(key_mtx);
        cv_keypress.wait(lk, []{
            return key_set.load(std::memory_order_seq_cst) == true;
        });
        if (key.load(std::memory_order_seq_cst) == 27){
            printf("Breaking from store image thread\n");
            break;
        }
        switch(key.load(std::memory_order_seq_cst)) {
            case int('a'): {
                printf("Type in name of the image(e.g. fig0), and press enter.\n");
                std::string img_name;
                std::getline(std::cin, img_name);
                img_name.append(".jpg");
                printf("Image file name: %s\n", img_name.c_str());
                cv::Mat img = getCurrentImg();
                break;
            }
            default: {
                printf("The key pressed is %d\n", key.load(std::memory_order_seq_cst));
                printf("To acquire image press 'a'\n");
                break;
            }
        } 
        key_set.store(false, std::memory_order_seq_cst);
        cv_keypress.notify_one();
    }
}


int main(int argc, char** argv) {

    std::thread detect_t(detectKey);
    std::thread store_t(storeImage);

    if (detect_t.joinable()) {
        detect_t.join();
    }
    if (store_t.joinable()) {
        store_t.join();
    }
    return EXIT_SUCCESS;
}

cv::Mat getCurrentImg() {

}

void streamImg() {
    while(key.load(std::memory_order_seq_cst) != 27) {
        std::lock_guard<std::mutex> lk(cur_img.mtx);
        
    }
}

