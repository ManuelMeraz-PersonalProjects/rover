//
// Created by manny on 3/30/20.
//

#include "gpio.hpp"

#include <wiringPi.h>

auto gpio::is_ready() -> bool
{
   return wiringPiSetup() != -1;
}

auto gpio::set_pin_mode(uint8_t pin_number, uint8_t mode) -> void
{
   pinMode(pin_number, static_cast<int>(mode));
}
auto gpio::sleep(std::chrono::microseconds duration) -> void
{
   ::delay(static_cast<int>(duration.count()));
}
