//
// Created by manny on 3/31/20.
//

#ifndef GPIO_BRIDGE_PWM_HPP
#define GPIO_BRIDGE_PWM_HPP
#include <cstdint>
#include <memory>

namespace gpio::pwm {
enum class Mode : int { OUTPUT = 4, SOFT_OUTPUT = 6, TONE_OUTPUTOFF = 8, OFF = 0 };

void duty_cycle(uint8_t pin_number, uint8_t duty_cycle);
void clock(uint8_t hz);
void range(uint16_t range);

class Pin
{
 public:
   using uPtr = std::unique_ptr<Pin>;
   using sPtr = std::shared_ptr<Pin>;

   Pin() = delete;
   Pin(const Pin&) = delete;
   Pin(Pin&&) = delete;
   auto operator=(const Pin&) -> Pin& = delete;
   auto operator=(Pin &&) -> Pin& = delete;

   explicit Pin(uint8_t pin_number, Mode mode = Mode::OUTPUT);
   ~Pin();

   [[nodiscard]] auto mode() const -> Mode;
   void mode(Mode mode);

   [[nodiscard]] auto duty_cycle() const -> uint8_t;
   void duty_cycle(uint8_t duty_cycle);

 private:
   uint8_t m_pin_number{};
   uint8_t m_duty_cycle{};

   Mode m_mode{};
};
} // namespace gpio::pwm

#endif // GPIO_BRIDGE_PWM_HPP
