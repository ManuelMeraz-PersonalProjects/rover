//
// Created by manny on 3/31/20.
//

#ifndef GPIO_BRIDGE_PIN_HPP
#define GPIO_BRIDGE_PIN_HPP

#include "gpio.hpp"

#include <cstdint>

namespace gpio {
class Pin
{
 public:
   explicit Pin(uint8_t pin_number, Mode mode = Mode::OUTPUT);
   ~Pin();

   [[nodiscard]] auto pin_number() const -> uint8_t;

   [[nodiscard]] auto mode() const -> Mode;
   auto set_mode(Mode mode);

 private:
   uint8_t m_pin_number;
   Mode m_mode;
};
} // namespace gpio

#endif // GPIO_BRIDGE_PIN_HPP
