//
// Created by manny on 3/31/20.
//

#include "Pin.hpp"
gpio::Pin::Pin(uint8_t pin_number, gpio::Mode mode) :
   m_pin_number(pin_number), m_mode(mode)
{
   gpio::set_pin_mode(m_pin_number, m_mode);
}
gpio::Pin::~Pin()
{
   gpio::set_pin_mode(m_pin_number, gpio::Mode::OFF);
}
auto gpio::Pin::pin_number() const -> uint8_t
{
   return m_pin_number;
}
auto gpio::Pin::mode() const -> gpio::Mode
{
   return m_mode;
}

auto gpio::Pin::set_mode(gpio::Mode mode)
{
   return nullptr;
}
