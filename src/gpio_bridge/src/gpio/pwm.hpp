//
// Created by manny on 3/31/20.
//

#ifndef GPIO_BRIDGE_PWM_HPP
#define GPIO_BRIDGE_PWM_HPP
#include <cstdint>

namespace gpio::pwm {
enum class Mode : int { OUTPUT = 4, SOFT_OUTPUT = 6, TONE_OUTPUTOFF = 8, OFF = 0 };

void set_duty_cycle(uint8_t pin_number, uint8_t duty_cycle);
void set_clock(uint8_t hz);
void set_range(uint16_t range);

class Pin
{
 public:
   explicit Pin(uint8_t pin_number, Mode mode = Mode::OUTPUT);
   ~Pin();

   [[nodiscard]] auto mode() const -> Mode;
   auto set_mode(Mode mode);

   [[nodiscard]] auto duty_cycle() const -> uint8_t;
   void set_duty_cycle(uint8_t duty_cycle);

 private:
   uint8_t m_pin_number{};
   uint8_t m_duty_cycle{};

   Mode m_mode{};
};
} // namespace gpio::pwm

#endif // GPIO_BRIDGE_PWM_HPP
