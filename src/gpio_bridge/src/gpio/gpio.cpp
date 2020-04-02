//
// Created by manny on 3/30/20.
//

#include "gpio.hpp"

#include <wiringPi.h>

auto gpio::is_ready() -> bool
{
   return wiringPiSetup() != -1;
}

void gpio::pin_mode(uint8_t pin_number, uint8_t mode)
{
   pinMode(pin_number, static_cast<int>(mode));
}
void gpio::sleep(std::chrono::milliseconds duration)
{
   ::delay(static_cast<int>(duration.count()));
}
