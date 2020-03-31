//
// Created by manny on 3/30/20.
//

#include "gpio.hpp"

#include <wiringPi.h>

auto gpio::is_ready() -> bool
{
   return wiringPiSetup() != -1;
}
auto gpio::set_pin_mode(uint8_t pin_number, gpio::Mode mode)
{
   pinMode(pin_number, mode);
}

auto gpio::set_duty_cycle(uint8_t pin_number, uint8_t duty_cycle)
{
   pwmWrite(pin_number, duty_cycle);
}
