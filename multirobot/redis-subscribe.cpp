#include "sw/redis++/redis++.h"
#include "sw/redis++/subscriber.h"

using namespace sw;
int main(int argc, char** argv) {
    auto red = redis::Redis("tcp://127.0.0.1:6379");
    auto sub = red.subscriber();
    sub.subscribe("task");
    while(sub.)

}