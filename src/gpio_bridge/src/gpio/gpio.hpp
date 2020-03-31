//
// Created by manny on 3/30/20.
//

#ifndef GPIO_BRIDGE_GPIO_HPP
#define GPIO_BRIDGE_GPIO_HPP

#include <cstdint>

namespace gpio {

enum Mode : int {
   INPUT,
   OUTPUT,
   INPUT_PULLUP,
   INPUT_PULLDOWN,
   PWM_OUTPUT,
   GPIO_CLOCK,
   SOFT_PWM_OUTPUT,
   SOFT_TONE_OUTPUT,
   PWM_TONE_OUTPUT,
   OFF = INPUT
};

auto is_ready() -> bool;
auto set_pin_mode(uint8_t pin_number, Mode mode) -> void;
auto set_duty_cycle(uint8_t pin_number, uint8_t duty_cycle) -> void;
} // namespace gpio

#endif // GPIO_BRIDGE_GPIO_HPP
