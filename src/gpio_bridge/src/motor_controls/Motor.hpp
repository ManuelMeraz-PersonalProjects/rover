//
// Created by manny on 3/30/20.
//

#ifndef GPIO_BRIDGE_MOTOR_HPP
#define GPIO_BRIDGE_MOTOR_HPP

#include <gpio/gpio.hpp>
namespace motor_controls {

enum class Direction { FORWARD, REVERSE };

class Motor
{
 public:
   explicit Motor(const gpio::digital::Pin& dir_pin, const gpio::pwm::Pin& pwm_pin);
   ~Motor();

   [[nodiscard]] auto direction() const -> Direction;
   [[nodiscard]] auto duty_cycle() const -> uint8_t;

   void stop();
   void actuate(Direction direction, uint8_t duty_cycle);

 private:
   static constexpr std::chrono::milliseconds DELTA_SLEEP_TIME{100};
   static constexpr uint8_t DUTY_CYCLE_DELTA = 10;

   Direction m_direction;
   uint8_t m_duty_cycle;
   gpio::digital::Pin m_dir_pin;
   gpio::pwm::Pin m_pwm_pin;
};

} // namespace motor_controls
#endif // GPIO_BRIDGE_MOTOR_HPP
