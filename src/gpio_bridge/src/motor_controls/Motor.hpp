//
// Created by manny on 3/30/20.
//

#ifndef GPIO_BRIDGE_MOTOR_HPP
#define GPIO_BRIDGE_MOTOR_HPP

#include <chrono>
#include <memory>
#include <odroid/gpio.hpp>
#include <optional>

namespace motor_controls {
enum class Direction { FORWARD, REVERSE };

class Motor
{
 public:
   using uPtr = std::unique_ptr<Motor>;
   using sPtr = std::shared_ptr<Motor>;

   Motor() = delete;
   Motor(const Motor&) = delete;
   Motor(Motor&&) = delete;
   auto operator=(const Motor&) -> Motor& = delete;
   auto operator=(Motor &&) -> Motor& = delete;

   explicit Motor(gpio::digital::DigitalPin& dir_pin, gpio::pwm::PWMPin& pwm_pin);
   ~Motor();

   [[nodiscard]] auto direction() const -> Direction;
   [[nodiscard]] auto duty_cycle() const -> uint8_t;

   void stop();
   void actuate(Direction direction,
                uint8_t duty_cycle,
                std::optional<std::chrono::milliseconds> time = std::nullopt);

 private:
   /**
    * Setting this any lower than 40 causes motor faults when free spinning
    */
   static constexpr std::chrono::milliseconds DELTA_SLEEP_TIME{100};

   /**
    * Setting this to anything greater 14 causes motors to fault when free
    * spinning with 50 ms sleep time
    */
   static constexpr uint8_t DUTY_CYCLE_DELTA = 4;

   Direction m_direction;
   uint8_t m_duty_cycle;
   gpio::digital::DigitalPin& m_dir_pin;
   gpio::pwm::PWMPin& m_pwm_pin;
};

} // namespace motor_controls
#endif // GPIO_BRIDGE_MOTOR_HPP
