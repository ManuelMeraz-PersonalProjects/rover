#include <wiringPi.h>
#include <cstdio>
#include <iostream>

void run_motors(int pwm_pin, int dir_pin, int dir_mode, int duty_cycle) {
    digitalWrite(dir_pin, dir_mode);
    for (int i = 0; i <= duty_cycle; i+=10) {
        pwmWrite(pwm_pin, i);  // set the Duty Cycle for this range.
        delay(100);
    }

    for (int i = duty_cycle; i >= 0; i-=10) {
        pwmWrite(pwm_pin, i);  // set the Duty Cycle for this range.
        delay(100);
    }
}

auto main() -> int
{
   // PWM1 is wiringPi Pin #1 or gpio #18
   // we choose this pin since it supports PWM as
   // PWM is not supported by any other gpio pins.
   constexpr int PWM1 = 23;
   constexpr int DIR1 = 21;

   constexpr int PWM2 = 24;
   constexpr int DIR2 = 22;

   if (wiringPiSetup() == -1) {
      printf("Setup wiringPi Failed!\n");
      return -1;
   }

   pinMode(DIR1, OUTPUT);
   pinMode(DIR2, OUTPUT);
   pinMode(PWM1, PWM_OUTPUT);
   pinMode(PWM2, PWM_OUTPUT);
   pwmSetClock(128); // 19.2 Mhz divided by 3840 is 5 Khz.

   constexpr int range = 100;
   pwmSetRange(range); // range is 2500 counts to give us half second.
   delay(1);           // delay a moment to let hardware settings settle.

   constexpr int DUTY_CYCLE = 500;
   std::cout << "Running forward" << std::endl;
   run_motors(PWM1, DIR1, LOW, DUTY_CYCLE);
   run_motors(PWM2, DIR2, LOW, DUTY_CYCLE);

   std::cout << "Running reverse" << std::endl;
   run_motors(PWM1, DIR1, HIGH, DUTY_CYCLE);
   run_motors(PWM2, DIR2, HIGH, DUTY_CYCLE);

   pinMode(DIR1, 0);
   pinMode(DIR2, 0);
   pinMode(PWM1, 0);
   pinMode(PWM2, 0);
   return 0;
}
