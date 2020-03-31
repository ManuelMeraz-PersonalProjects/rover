//
// Created by manny on 3/30/20.
//

#ifndef GPIO_BRIDGE_PWMPIN_HPP
#define GPIO_BRIDGE_PWMPIN_HPP

#include "Pin.hpp"
#include "gpio.hpp"

#include <cstdint>

namespace gpio {
class PWMPin : public Pin
{
 public:
   explicit PWMPin(uint8_t pin_number, Mode mode = Mode::PWM_OUTPUT);
   ~PWMPin();

   [[nodiscard]] auto duty_cycle() const -> uint8_t;
   auto set_duty_cycle(uint8_t duty_cycle);

 private:
   uint8_t m_pin_number;
   uint8_t m_duty_cycle;
   Mode m_mode;
};

} // namespace gpio
#endif // GPIO_BRIDGE_PWMPIN_HPP
