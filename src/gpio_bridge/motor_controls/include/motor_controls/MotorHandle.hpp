#ifndef GPIO_BRIDGE_MOTOR_CONTROLS_MOTORHANDLE_HPP
#define GPIO_BRIDGE_MOTOR_CONTROLS_MOTORHANDLE_HPP

#include <hardware_interface/joint_command_handle.hpp>
#include <hardware_interface/joint_state_handle.hpp>
#include <hardware_interface/operation_mode_handle.hpp>
#include <memory>

namespace motor_controls {
struct MotorHandle
{
   using sharedPtr = std::shared_ptr<MotorHandle>;

   double position{};
   double velocity{};
   double effort{};
   double command{};
   hardware_interface::JointStateHandle joint_state_handle{};
   hardware_interface::JointCommandHandle joint_command_handle{};
};
} // namespace motor_controls

#endif // GPIO_BRIDGE_MOTOR_CONTROLS_MOTORHANDLE_HPP
