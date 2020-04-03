#include "MotorController.hpp"
#include "gpio/gpio.hpp"
#include "motor_controls/Motor.hpp"

#include <chrono>
#include <iostream>
#include <thread>

using namespace std::chrono_literals;

auto main() -> int
{
   if (!gpio::is_ready()) {
      std::cout << "Setup wiringPi Failed!\n";
      return 1;
   }

   motor_controls::MotorController controller;

   constexpr uint8_t DUTY_CYCLE = 100;

   const auto move_left_motor = [&controller](motor_controls::Direction direction,
                                              std::chrono::milliseconds time) {
      controller.left().actuate(direction, DUTY_CYCLE, time);
   };

   const auto move_right_motor = [&controller](motor_controls::Direction direction,
                                               std::chrono::milliseconds time) {
      controller.right().actuate(direction, DUTY_CYCLE, time);
   };

   std::thread move_left(move_left_motor, motor_controls::Direction::FORWARD, 3s);
   std::thread move_right(move_right_motor, motor_controls::Direction::FORWARD, 3s);

   move_left.join();
   move_right.join();

   return 0;
}
