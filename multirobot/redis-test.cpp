#include "sw/redis++/redis++.h"
#include "sw/redis++/subscriber.h"
#include <thread>

using namespace sw;
int main(int argc, char** argv) {
    auto red = redis::Redis("tcp://127.0.0.1:6379");
    redis::StringView taskid;
    taskid = "3";
    for (int i = 0; i < 100; i++) {
        red.publish("task", std::to_string(i));
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    // red.publish("task", taskid);
}