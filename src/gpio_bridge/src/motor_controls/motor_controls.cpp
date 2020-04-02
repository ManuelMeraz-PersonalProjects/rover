#include "gpio/gpio.hpp"
#include "motor_controls/Motor.hpp"

#include <chrono>
#include <iostream>

using namespace std::chrono_literals;
namespace digital = gpio::digital;
namespace pwm = gpio::pwm;

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
   pwm::clock(CLOCK_HZ);

   constexpr int RANGE = 100;
   pwm::range(RANGE); // range is 2500 counts to give us half second.

   motor_controls::Motor left_motor(dir2_pin, pwm2_pin);
   motor_controls::Motor right_motor(dir1_pin, pwm1_pin);

   gpio::sleep(1ms);

   constexpr uint8_t DUTY_CYCLE = 100;
   left_motor.actuate(motor_controls::Direction::FORWARD, DUTY_CYCLE, 3s);
   right_motor.actuate(motor_controls::Direction::FORWARD, DUTY_CYCLE, 3s);

   left_motor.actuate(motor_controls::Direction::REVERSE, DUTY_CYCLE, 3s);
   right_motor.actuate(motor_controls::Direction::REVERSE, DUTY_CYCLE, 3s);

   return 0;
}
