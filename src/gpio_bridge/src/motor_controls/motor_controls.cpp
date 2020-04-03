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
   motor_controls::Command command{motor_controls::Direction::FORWARD, DUTY_CYCLE};
   controller.actuate(command);

   command.direction = motor_controls::Direction::REVERSE;
   controller.actuate(command);

   controller.stop();
}
