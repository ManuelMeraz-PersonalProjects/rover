//
// Created by manny on 3/31/20.
//

#ifndef GPIO_BRIDGE_DIGITAL_HPP
#define GPIO_BRIDGE_DIGITAL_HPP

#include <cstdint>

namespace gpio::digital {
enum class Mode : uint8_t { INPUT = 0, OUTPUT = 1, OFF = 0 };
enum class Write : uint8_t { LOW = 0, HIGH = 1 };

auto write(uint8_t pin_number, digital::Write mode) -> void;

class Pin
{
 public:
   explicit Pin(uint8_t pin_number, Mode mode = Mode::OUTPUT);
   ~Pin();

   [[nodiscard]] auto mode() const -> Mode;
   auto set_mode(Mode mode) -> void;
   auto write(Write mode) -> void;

 private:
   uint8_t m_pin_number{};
   Mode m_mode{};
};
} // namespace gpio::digital

#endif // GPIO_BRIDGE_DIGITAL_HPP
