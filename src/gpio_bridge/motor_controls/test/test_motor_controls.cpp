#include "motor_controls/motor_controls.hpp"

#include <chrono>
#include <iostream>
#include <odroid/gpio.hpp>

using namespace std::chrono_literals;

auto main() -> int
{
   if (!gpio::setup()) {
      std::cout << "Setup Odroid GPIO Failed!" << std::endl;
      return 1;
   }

   auto& controller = motor_controls::MotorController::get();

   constexpr uint8_t DUTY_CYCLE = 100;

   std::cout << "Actuating motors forward for 10 seconds at 100% duty cycle" << std::endl;
   motor_controls::Command command{motor_controls::Direction::FORWARD, DUTY_CYCLE, 10s};
   controller.actuate(command);

   std::cout << "Actuating motors reverse for 10 seconds at 100% duty cycle" << std::endl;
   command.direction = motor_controls::Direction::REVERSE;
   controller.actuate(command);

   controller.stop();
}
