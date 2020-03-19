#include <wiringPi.h>
#include <iostream>

int main() {
    std::cout << "Odroid N2 wiringPi test program\n";

    if (wiringPiSetupGpio() == -1) {
        wiringPiFailure(WPI_FATAL, "Failed to setup wiring pi.");
        return 1;
    }

    pinMode(0, PWM_OUTPUT);
    pwmSetClock(2);
    pwmSetRange(10);
    pwmWrite(0, 5);

    for (;;) delay(1000);
}