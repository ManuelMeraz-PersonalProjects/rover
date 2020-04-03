//
// Created by manny on 3/31/20.
//

#include "pwm.hpp"

#include "gpio.hpp"

#include <wiringPi.h>

void gpio::pwm::duty_cycle(uint8_t pin_number, uint8_t duty_cycle)
{
   pwmWrite(pin_number, duty_cycle);
}

void gpio::pwm::clock(uint8_t hz)
{
   pwmSetClock(hz);
}
void gpio::pwm::range(uint16_t range)
{
   pwmSetRange(range);
}

gpio::pwm::Pin::Pin(uint8_t pin_number, Mode mode) : m_pin_number(pin_number), m_mode(mode)
{
   gpio::pin_mode(m_pin_number, static_cast<uint8_t>(m_mode));
   duty_cycle(0);
}

gpio::pwm::Pin::~Pin()
{
   duty_cycle(0);
   gpio::pin_mode(m_pin_number, static_cast<uint8_t>(Mode::OFF));
}

auto gpio::pwm::Pin::mode() const -> Mode
{
   return m_mode;
}

void gpio::pwm::Pin::mode(Mode mode)
{
   m_mode = mode;
   gpio::pin_mode(m_pin_number, static_cast<uint8_t>(m_mode));
}

auto gpio::pwm::Pin::duty_cycle() const -> uint8_t
{
   return m_duty_cycle;
}

void gpio::pwm::Pin::duty_cycle(uint8_t duty_cycle)
{
   m_duty_cycle = duty_cycle;
   gpio::pwm::duty_cycle(m_pin_number, duty_cycle);
}
