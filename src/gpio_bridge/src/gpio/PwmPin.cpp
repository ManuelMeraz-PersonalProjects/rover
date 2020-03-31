//
// Created by manny on 3/30/20.
//

#include "PwmPin.hpp"

#include "gpio.hpp"

#include <algorithm>
#include <array>

gpio::PWMPin::PWMPin(uint8_t pin_number, Mode mode) :
   Pin(pin_number, mode), m_duty_cycle(mode)
{
   gpio::set_pin_mode(m_pin_number, m_mode);
}

gpio::PWMPin::~PWMPin()
{
   gpio::set_pin_mode(m_pin_number, Mode::OFF);
}

auto gpio::PWMPin::duty_cycle() const -> uint8_t
{
   return m_duty_cycle;
}

auto gpio::PWMPin::set_duty_cycle(uint8_t duty_cycle)
{
   m_duty_cycle = duty_cycle;
   gpio::set_duty_cycle(m_pin_number, duty_cycle);
}
