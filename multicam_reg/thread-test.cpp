#include <thread>
#include <iostream>

class X {
public:
    void printSomething() {
        printf("Print Something.\n");
    }

    static void printStatic() {
        printf("Print Something static.\n");
    }
    void printThread() {
        std::thread t(&X::printSomething, this); // for a non static member, will need to pass in the reference to 
        if (t.joinable()) {
            t.join();
        }
        std::thread t2(printStatic);
        if (t2.joinable()) {
            t2.join();
        }
        return;
    }
    void addThread() {
        auto add = [this]() {
            this->b++;
            printf("b is %.3f.\n", this->b);
        };
        std::thread t(add);
        if (t.joinable()) {
            t.join();
        }
    }
private:
    double b{100};
};

int main(int argc, char** argv) {
    X x;
    x.printThread();
    X::printStatic();
    x.addThread();
    return 0;
}