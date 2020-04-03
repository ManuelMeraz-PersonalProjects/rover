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

   const auto move_motors = [&controller, &DUTY_CYCLE](auto direction, auto time) {
      controller.left().actuate(direction, DUTY_CYCLE);
      controller.right().actuate(direction, DUTY_CYCLE);
      gpio::sleep(time);
      controller.left().stop();
      controller.right().stop();
   };

   move_motors(motor_controls::Direction::FORWARD, 3s);
   move_motors(motor_controls::Direction::REVERSE, 3s);

   return 0;
}
