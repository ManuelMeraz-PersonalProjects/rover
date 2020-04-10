#include "motor_controls/MotorController.hpp"

#include <thread>

using namespace std::chrono_literals;
namespace digital = gpio::digital;
namespace pwm = gpio::pwm;

namespace {

// globals
std::shared_ptr<motor_controls::MotorController> g_controller{nullptr};

constexpr int DIR1_WIRING_PI_PIN{21};
constexpr int DIR2_WIRING_PI_PIN{22};
constexpr int PWM1_WIRING_PI_PIN{23};
constexpr int PWM2_WIRING_PI_PIN{24};
} // namespace

motor_controls::MotorController::MotorController()
{
   auto& dir1_pin = gpio::get<digital::DigitalPin>(DIR1_WIRING_PI_PIN, digital::Mode::OUTPUT);
   auto& dir2_pin = gpio::get<digital::DigitalPin>(DIR2_WIRING_PI_PIN, digital::Mode::OUTPUT);
   auto& pwm1_pin = gpio::get<pwm::PWMPin>(PWM1_WIRING_PI_PIN, pwm::Mode::OUTPUT);
   auto& pwm2_pin = gpio::get<pwm::PWMPin>(PWM2_WIRING_PI_PIN, pwm::Mode::OUTPUT);

   m_left_motor = std::make_unique<motor_controls::Motor>(dir2_pin, pwm2_pin);
   m_right_motor = std::make_unique<motor_controls::Motor>(dir1_pin, pwm1_pin);
}

hardware_interface::hardware_interface_ret_t motor_controls::MotorController::init()
{
   constexpr std::array<const char*, 2> joint_names{"left_wheels", "right_wheels"};

   size_t i = 0;
   for (auto& joint_name : joint_names) {

      m_joint_states[i] = hardware_interface::JointStateHandle(
         joint_name, &m_positions[i], &m_velocities[i], &m_efforts[i]);
      if (register_joint_state_handle(&m_joint_states[i]) != hardware_interface::HW_RET_OK) {
         throw std::runtime_error("unable to register " + m_joint_states[i].get_name());
      }

      m_joint_commands[i] = hardware_interface::JointCommandHandle(joint_name, &m_commands[i]);
      if (register_joint_command_handle(&m_joint_commands[i]) != hardware_interface::HW_RET_OK) {
         throw std::runtime_error("unable to register " + m_joint_commands[i].get_name());
      }

      m_operation_modes[i] = hardware_interface::OperationMode(true);
      std::stringstream mode_stream;
      mode_stream << joint_name << "_mode";
      m_operation_modes_handles[i] =
         hardware_interface::OperationModeHandle(mode_stream.str(), &m_operation_modes[i]);
      if (register_operation_mode_handle(&m_operation_modes_handles[i]) !=
          hardware_interface::HW_RET_OK) {
         throw std::runtime_error("unable to register " + m_operation_modes_handles[i].get_name());
      }

      ++i;
   }

   return hardware_interface::HW_RET_OK;
}

hardware_interface::hardware_interface_ret_t motor_controls::MotorController::read()
{
   m_efforts[0] = m_left_motor->duty_cycle();
   m_efforts[1] = m_right_motor->duty_cycle();
   // do robot specific stuff to update the pos_, vel_, eff_ arrays

   return hardware_interface::HW_RET_OK;
}

hardware_interface::hardware_interface_ret_t motor_controls::MotorController::write()
{
   const double left_command_value = m_commands[0];
   const double right_command_value = m_commands[1];

   const Command left_command{left_command_value > 0 ? Direction::FORWARD : Direction::REVERSE,
                              static_cast<uint8_t>(std::abs(left_command_value))};

   const Command right_command{right_command_value > 0 ? Direction::FORWARD : Direction::REVERSE,
                               static_cast<uint8_t>(std::abs(right_command_value))};

   actuate(left_command, right_command);
   // do robot specific stuff to update the pos_, vel_, eff_ arrays
   return hardware_interface::HW_RET_OK;
}

void motor_controls::MotorController::actuate(const motor_controls::Command& both_command)
{
   const auto& left_command = both_command;
   const auto& right_command = both_command;
   actuate(left_command, right_command);
}

void motor_controls::MotorController::actuate(const motor_controls::Command& left_command,
                                              const motor_controls::Command& right_command)

{
   const auto actuate_left_motor = [this](const motor_controls::Command& command) {
      m_left_motor->actuate(command.direction, command.duty_cycle, command.time);
   };

   const auto actuate_right_motor = [this](const motor_controls::Command& command) {
      m_right_motor->actuate(command.direction, command.duty_cycle, command.time);
   };

   std::thread move_left(actuate_left_motor, left_command);
   std::thread move_right(actuate_right_motor, right_command);

   move_left.join();
   move_right.join();
}

void motor_controls::MotorController::stop()
{
   const auto stop_left_motor = [this]() { m_left_motor->stop(); };

   const auto stop_right_motor = [this]() { m_right_motor->stop(); };

   std::thread stop_left(stop_left_motor);
   std::thread stop_right(stop_right_motor);

   stop_left.join();
   stop_right.join();
}

auto motor_controls::MotorController::get() -> motor_controls::MotorController&
{
   return *getPtr();
}

auto motor_controls::MotorController::getPtr() -> std::shared_ptr<MotorController>
{
   if (g_controller == nullptr) {
      g_controller = std::make_shared<MotorController>();
   }

   return g_controller;
}
