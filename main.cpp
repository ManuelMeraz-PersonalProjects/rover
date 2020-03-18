#include <iostream>
#include <wiringPi.h>

int main() {
    std::cout << "Hello, World!" << std::endl;
    wiringPiFailure(4, "hello");
    return 0;
}
