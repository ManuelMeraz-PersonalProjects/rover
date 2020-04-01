//
// Created by manny on 3/31/20.
//

#include "digital.hpp"

#include "gpio.hpp"

#include <wiringPi.h>

auto gpio::digital::write(uint8_t pin_number, Write mode) -> void
{
   digitalWrite(pin_number, static_cast<int>(mode));
}

gpio::digital::Pin::Pin(uint8_t pin_number, Mode mode) : m_pin_number(pin_number), m_mode(mode)
{
   gpio::set_pin_mode(m_pin_number, static_cast<uint8_t>(m_mode));
}

gpio::digital::Pin::~Pin()
{
   gpio::set_pin_mode(m_pin_number, static_cast<uint8_t>(Mode::OFF));
}

auto gpio::digital::Pin::mode() const -> Mode
{
   return m_mode;
}

auto gpio::digital::Pin::set_mode(Mode mode) -> void
{
   m_mode = mode;
   gpio::set_pin_mode(m_pin_number, static_cast<uint8_t>(m_mode));
}

auto gpio::digital::Pin::write(Write mode) -> void
{
   digitalWrite(m_pin_number, static_cast<int>(mode));
}
