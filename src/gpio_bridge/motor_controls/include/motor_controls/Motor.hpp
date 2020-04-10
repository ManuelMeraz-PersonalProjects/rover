//
// Created by manny on 3/30/20.
//

#ifndef GPIO_BRIDGE_MOTOR_HPP
#define GPIO_BRIDGE_MOTOR_HPP

#include "MotorHandle.hpp"

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

   explicit Motor(std::string name, gpio::digital::DigitalPin& dir_pin, gpio::pwm::PWMPin& pwm_pin);
   ~Motor();

   [[nodiscard]] auto direction() const -> Direction;
   [[nodiscard]] auto duty_cycle() const -> uint8_t;
   [[nodiscard]] auto handle() const -> MotorHandle::sharedPtr;

   void stop();
   void actuate(Direction direction,
                uint8_t duty_cycle,
                std::optional<std::chrono::milliseconds> time = std::nullopt);

 private:
   static constexpr std::chrono::milliseconds DELTA_SLEEP_TIME{50};
   static constexpr uint8_t DUTY_CYCLE_DELTA = 10;

   Direction m_direction{};
   uint8_t m_duty_cycle{};

   gpio::digital::DigitalPin& m_dir_pin;
   gpio::pwm::PWMPin& m_pwm_pin;

   std::string m_name{};
   MotorHandle::sharedPtr m_handle{};
};

} // namespace motor_controls
#endif // GPIO_BRIDGE_MOTOR_HPP
