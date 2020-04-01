#include "gpio/gpio.hpp"

#include <chrono>
#include <iostream>

using namespace std::chrono_literals;
namespace digital = gpio::digital;
namespace pwm = gpio::pwm;

void run_motors(pwm::Pin& pwm_pin, digital::Pin& dir_pin, digital::Write write_mode,
                const uint8_t max_duty_cycle)
{
   dir_pin.write(write_mode);

   constexpr int DUTY_CYCLE_DELTA = 10;
   for (int duty_cycle = 0; duty_cycle <= max_duty_cycle; duty_cycle += DUTY_CYCLE_DELTA) {
      pwm_pin.set_duty_cycle(duty_cycle);
      gpio::sleep(100ms);
   }

   for (int duty_cycle = max_duty_cycle; duty_cycle >= max_duty_cycle;
        duty_cycle -= DUTY_CYCLE_DELTA) {
      pwm_pin.set_duty_cycle(duty_cycle);
      gpio::sleep(100ms);
   }
}

auto main() -> int
{
   if (!gpio::is_ready()) {
      std::cout << "Setup wiringPi Failed!\n";
      return 1;
   }

   constexpr int DIR1_WIRING_PI_PIN = 21;
   digital::Pin dir1_pin(DIR1_WIRING_PI_PIN, digital::Mode::OUTPUT);

   constexpr int DIR2_WIRING_PI_PIN = 22;
   digital::Pin dir2_pin(DIR2_WIRING_PI_PIN, digital::Mode::OUTPUT);

   constexpr int PWM1_WIRING_PI_PIN = 23;
   pwm::Pin pwm1_pin(PWM1_WIRING_PI_PIN, pwm::Mode::OUTPUT);

   constexpr int PWM2_WIRING_PI_PIN = 24;
   pwm::Pin pwm2_pin(PWM2_WIRING_PI_PIN, pwm::Mode::OUTPUT);

   constexpr uint8_t CLOCK_HZ = 128;
   pwm::set_clock(CLOCK_HZ);

   constexpr int RANGE = 100;
   pwm::set_range(RANGE); // range is 2500 counts to give us half second.

   gpio::sleep(1ms);

   constexpr uint8_t DUTY_CYCLE = 100;
   std::cout << "Running forward" << std::endl;
   run_motors(pwm1_pin, dir1_pin, digital::Write::LOW, DUTY_CYCLE);
   run_motors(pwm2_pin, dir2_pin, digital::Write::LOW, DUTY_CYCLE);

   std::cout << "Running reverse" << std::endl;
   run_motors(pwm1_pin, dir1_pin, digital::Write::HIGH, DUTY_CYCLE);
   run_motors(pwm2_pin, dir2_pin, digital::Write::HIGH, DUTY_CYCLE);
   return 0;
}
