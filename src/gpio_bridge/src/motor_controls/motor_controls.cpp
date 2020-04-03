#include "MotorController.hpp"
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
   //
   //   constexpr int DIR1_WIRING_PI_PIN = 21;
   //   auto dir1_pin = std::make_shared<digital::Pin>(DIR1_WIRING_PI_PIN, digital::Mode::OUTPUT);
   //
   //   constexpr int DIR2_WIRING_PI_PIN = 22;
   //   auto dir2_pin = std::make_shared<digital::Pin>(DIR2_WIRING_PI_PIN, digital::Mode::OUTPUT);
   //
   //   constexpr int PWM1_WIRING_PI_PIN = 23;
   //   auto pwm1_pin = std::make_shared<pwm::Pin>(PWM1_WIRING_PI_PIN, pwm::Mode::OUTPUT);
   //
   //   constexpr int PWM2_WIRING_PI_PIN = 24;
   //   auto pwm2_pin = std::make_shared<pwm::Pin>(PWM2_WIRING_PI_PIN, pwm::Mode::OUTPUT);
   //
   //   constexpr uint8_t CLOCK_HZ = 128;
   //   pwm::clock(CLOCK_HZ);
   //
   //   constexpr int RANGE = 100;
   //   pwm::range(RANGE); // range is 2500 counts to give us half second.

   motor_controls::MotorController controller;
   //  auto left_motor = std::make_shared<motor_controls::Motor>(std::move(dir2_pin),
   //  std::move(pwm2_pin));
   //   auto right_motor = std::make_shared<motor_controls::Motor>(std::move(dir1_pin),
   //   std::move(pwm1_pin));

   constexpr uint8_t DUTY_CYCLE = 100;
   controller.m_left_motor->actuate(motor_controls::Direction::FORWARD, DUTY_CYCLE, 3s);
   //   left_motor->actuate(motor_controls::Direction::FORWARD, DUTY_CYCLE, 3s);
   //   controller.m_right_motor.actuate(motor_controls::Direction::FORWARD, DUTY_CYCLE, 3s);
   //
   //   controller.m_left_motor.actuate(motor_controls::Direction::REVERSE, DUTY_CYCLE, 3s);
   //   controller.m_right_motor.actuate(motor_controls::Direction::REVERSE, DUTY_CYCLE, 3s);
   //
   return 0;
}
