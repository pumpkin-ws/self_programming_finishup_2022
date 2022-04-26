#include "sw/redis++/redis++.h"
#include "sw/redis++/subscriber.h"
#include "hiredis/hiredis.h"
#include <iostream>


// #define REDIS_ASSERT(condition, msg) \
//     sw::redis::test::redis_assert((condition), (msg), __FILE__, __LINE__)


using namespace sw;
int main(int argc, char** argv) {
    redis::ConnectionOptions opts;
    opts.host = "127.0.0.1";
    opts.port = 6379;
    opts.socket_timeout = std::chrono::milliseconds(100);

    auto red = redis::Redis(opts);
    auto sub = red.subscriber();
    auto msgs = {"msg1", "msg2"};
    std::string channel1 = "task";
    sub.on_message([](std::string channel, std::string msg) {
                        std::cout << "A new message is detected." << std::endl;
                        printf("Topic:%s.  Message:%s.\n", channel.c_str(), msg.c_str());
                    });
    sub.subscribe(channel1);
    while(true) {
        try{
            sub.consume();
        } catch (const redis::Error &err) {
            printf("An redis exception has occured. Error msg: %s\n", err.what());
            return EXIT_FAILURE;
        }
    }

}