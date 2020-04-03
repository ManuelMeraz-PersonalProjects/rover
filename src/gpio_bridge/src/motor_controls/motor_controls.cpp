#include "MotorController.hpp"
#include "gpio/gpio.hpp"
#include "motor_controls/Motor.hpp"

#include <chrono>
#include <iostream>

using namespace std::chrono_literals;

auto main() -> int
{
   if (!gpio::is_ready()) {
      std::cout << "Setup wiringPi Failed!\n";
      return 1;
   }

   motor_controls::MotorController controller;

   constexpr uint8_t DUTY_CYCLE = 100;
   controller.left().actuate(motor_controls::Direction::FORWARD, DUTY_CYCLE, 3s);
   controller.right().actuate(motor_controls::Direction::FORWARD, DUTY_CYCLE, 3s);

   controller.left().actuate(motor_controls::Direction::REVERSE, DUTY_CYCLE, 3s);
   controller.right().actuate(motor_controls::Direction::REVERSE, DUTY_CYCLE, 3s);
   return 0;
}
