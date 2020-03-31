//
// Created by manny on 3/30/20.
//

#include "PwmPin.hpp"

#include "gpio.hpp"

#include <algorithm>
#include <array>

constexpr gpio::PWMPin::PWMPin(uint8_t pin_number, Mode mode) :
   m_pin_number(pin_number), m_duty_cycle(0), m_mode(mode)
{
}

gpio::PWMPin::~PWMPin()
{
   gpio::set_pin_mode(m_pin_number, Mode::OFF);
}

auto gpio::PWMPin::pin_number() const -> uint8_t
{
   return m_pin_number;
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

auto gpio::PWMPin::mode() -> gpio::Mode
{
   return m_mode;
}
auto gpio::PWMPin::set_mode(gpio::Mode mode)
{
   m_mode = mode;
   gpio::set_pin_mode(m_pin_number, m_mode);
}
