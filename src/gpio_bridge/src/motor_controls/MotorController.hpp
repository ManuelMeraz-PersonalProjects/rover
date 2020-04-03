//
// Created by manny on 4/2/20.
//

#ifndef GPIO_BRIDGE_MOTORCONTROLLER_HPP
#define GPIO_BRIDGE_MOTORCONTROLLER_HPP

#include "Motor.hpp"
/*
 * Motor Controller | WiringPi | Physical Pin | Wire Color
 * -------------------------------------------------------
 * dir1             |21        | 31           | Blue
 * pwm1             |23        | 35           | Green
 * dir2             |22        | 33           | Yellow
 * pwm2             |24        | 37           | Red
 * gnd              | -        | 39           | Black
 */
namespace motor_controls {

struct Command
{
   Direction direction{Direction::FORWARD};
   uint8_t duty_cycle{0};
   std::optional<std::chrono::milliseconds> time{std::nullopt};
};

class MotorController
{
 public:
   using uPtr = std::unique_ptr<MotorController>;
   using sPtr = std::shared_ptr<MotorController>;

   MotorController(const MotorController&) = delete;
   MotorController(MotorController&&) = delete;
   auto operator=(const MotorController&) -> MotorController& = delete;
   auto operator=(MotorController &&) -> MotorController& = delete;

   static auto get() -> MotorController&;

   /**
    * \brief Apply same command to both motors.
    * \param both_command Command containing command for both motors.
    */
   void actuate(const Command& both_command);

   /**
    * \brief Apply individual command to each motor
    * \param both Command containing command for each motor
    */
   void actuate(const Command& left_command, const Command& right_command);

   /**
    * \brief Stop both motors.
    */
   void stop();

 private:
   MotorController();
   ~MotorController();

   Motor::uPtr m_left_motor{};
   Motor::uPtr m_right_motor{};
};
} // namespace motor_controls
#endif // GPIO_BRIDGE_MOTORCONTROLLER_HPP
