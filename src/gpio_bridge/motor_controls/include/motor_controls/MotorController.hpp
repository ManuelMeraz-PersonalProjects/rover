#ifndef GPIO_BRIDGE_MOTORCONTROLLER_HPP
#define GPIO_BRIDGE_MOTORCONTROLLER_HPP

#include "Motor.hpp"

#include <array>
#include <chrono>
#include <controller_interface/controller_interface.hpp>
/*
 * Motor Controller | WiringPi | Physical Pin | Wire Color
 * -------------------------------------------------------
 * dir1             |21        | 29           | Blue
 * dir2             |22        | 31           | Yellow
 * pwm1             |23        | 33           | Green
 * pwm2             |24        | 35           | Red
 * gnd              | -        | 39           | Black
 */

namespace motor_controls {
struct Command
{
   Direction direction{Direction::FORWARD};
   uint8_t duty_cycle{0};
   std::optional<std::chrono::milliseconds> time{std::nullopt};
};

class MotorController : public hardware_interface::RobotHardware
{
 public:
   MotorController();
   ~MotorController() override = default;
   MotorController(const MotorController&) = delete;
   MotorController(MotorController&&) = delete;
   auto operator=(const MotorController&) -> MotorController& = delete;
   auto operator=(MotorController &&) -> MotorController& = delete;

   static auto get() -> MotorController&;
   static auto getPtr() -> std::shared_ptr<MotorController>;

   HARDWARE_INTERFACE_PUBLIC
   hardware_interface::hardware_interface_ret_t init() override;

   HARDWARE_INTERFACE_PUBLIC
   hardware_interface::hardware_interface_ret_t read() override;

   HARDWARE_INTERFACE_PUBLIC
   hardware_interface::hardware_interface_ret_t write() override;

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
   Motor::uPtr m_left_motor{};
   Motor::uPtr m_right_motor{};

   std::mutex m_mutex;

   std::array<double, 2> m_positions{};
   std::array<double, 2> m_velocities{};
   std::array<double, 2> m_efforts{};
   std::array<double, 2> m_commands{};

   std::array<hardware_interface::JointStateHandle, 2> m_joint_states{};
   std::array<hardware_interface::JointCommandHandle, 2> m_joint_commands{};
   std::array<hardware_interface::OperationModeHandle, 2> m_operation_modes_handles{};
   std::array<hardware_interface::OperationMode, 2> m_operation_modes{};
};
} // namespace motor_controls
#endif // GPIO_BRIDGE_MOTORCONTROLLER_HPP
