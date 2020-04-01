//
// Created by manny on 3/30/20.
//

#ifndef GPIO_BRIDGE_GPIO_HPP
#define GPIO_BRIDGE_GPIO_HPP

#include "digital.hpp"
#include "pwm.hpp"

#include <chrono>
#include <cstdint>
namespace gpio {
auto is_ready() -> bool;
auto set_pin_mode(uint8_t pin_number, uint8_t mode) -> void;
auto sleep(std::chrono::microseconds duration) -> void;
} // namespace gpio

#endif // GPIO_BRIDGE_GPIO_HPP
