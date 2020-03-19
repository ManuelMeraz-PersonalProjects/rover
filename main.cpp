#include <wiringPi.h>
#include <cstdio>


int main() {
    // LEDPIN is wiringPi Pin #1 or GPIO #18
    // we choose this pin since it supports PWM as
    // PWM is not supported by any other GPIO pins.
    const int LEDPIN = 23;

    if (wiringPiSetup() == -1) {
        printf("Setup wiringPi Failed!\n");
        return -1;
    }

    pinMode(LEDPIN, PWM_OUTPUT);
    // set the PWM mode to Mark Space
    //pwmSetMode(PWM_MODE_MS);
    pwmSetMode(0);
    // set the clock divisor to reduce the 19.2 Mhz clock
    // to something slower, 5 Khz.
    // Range of pwmSetClock() is 2 to 4095.
    pwmSetClock(128);  // 19.2 Mhz divided by 3840 is 5 Khz.

    // set the PWM range which is the value for the range counter
    // which is decremented at the modified clock frequency.
    // in this case we are decrementing the range counter 5,000
    // times per second since the clock at 19.2 Mhz is being
    // divided by 3840 to give us 5 Khz.
    constexpr int range = 100;
    pwmSetRange(range);  // range is 2500 counts to give us half second.
    delay(1);   // delay a moment to let hardware settings settle.

    pwmWrite(LEDPIN, 50);  // set the Duty Cycle for this range.
    printf(" PWM Duty Cycle %d\n", 50);
    delay(5000);

    pwmWrite(LEDPIN, 0);  // set the Duty Cycle for this range.
    // cleanup the environment. set each pin to low
    // and set the mode to INPUT. These steps make sure
    // the equipment is safe to manipulate and prevents
    // possible short and equipment damage from energized pin.
    pinMode(LEDPIN, INPUT);
    digitalWrite(LEDPIN, LOW);

    return 0;
}
