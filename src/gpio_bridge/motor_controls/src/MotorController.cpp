#include "motor_controls/MotorController.hpp"

#include "motor_controls/MotorHandle.hpp"

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
   if (!gpio::setup()) {
      throw std::runtime_error("Setup Odroid GPIO Failed!");
   }

   auto& dir1_pin = gpio::get<digital::DigitalPin>(DIR1_WIRING_PI_PIN, digital::Mode::OUTPUT);
   auto& dir2_pin = gpio::get<digital::DigitalPin>(DIR2_WIRING_PI_PIN, digital::Mode::OUTPUT);
   auto& pwm1_pin = gpio::get<pwm::PWMPin>(PWM1_WIRING_PI_PIN, pwm::Mode::OUTPUT);
   auto& pwm2_pin = gpio::get<pwm::PWMPin>(PWM2_WIRING_PI_PIN, pwm::Mode::OUTPUT);

   m_left_motor = std::make_unique<motor_controls::Motor>("left_wheels", dir1_pin, pwm1_pin);
   m_right_motor = std::make_unique<motor_controls::Motor>("right_wheels", dir2_pin, pwm2_pin);
}

hardware_interface::hardware_interface_ret_t motor_controls::MotorController::init()
{

   const auto init_motor = [this](const MotorHandle::sPtr& handle) {
      if (register_joint_state_handle(&handle->joint_state_handle) !=
          hardware_interface::HW_RET_OK) {
         throw std::runtime_error("unable to register " + handle->joint_state_handle.get_name());
      }

      if (register_joint_command_handle(&handle->joint_command_handle) !=
          hardware_interface::HW_RET_OK) {
         throw std::runtime_error("unable to register " + handle->joint_command_handle.get_name());
      }
   };

   init_motor(m_left_motor->handle());
   init_motor(m_right_motor->handle());

   if (register_operation_mode_handle(&m_operation.handle) != hardware_interface::HW_RET_OK) {
      throw std::runtime_error("unable to register " + m_operation.handle.get_name());
   }
   return hardware_interface::HW_RET_OK;
}

hardware_interface::hardware_interface_ret_t motor_controls::MotorController::read()
{
   auto left_motor_handle = m_left_motor->handle();
   auto right_motor_handle = m_right_motor->handle();

   left_motor_handle->effort = m_left_motor->duty_cycle();
   right_motor_handle->effort = m_right_motor->duty_cycle();

   return hardware_interface::HW_RET_OK;
}

hardware_interface::hardware_interface_ret_t motor_controls::MotorController::write()
{
   auto left_motor_handle = m_left_motor->handle();
   auto right_motor_handle = m_right_motor->handle();

   const double left_command_value = left_motor_handle->command;
   const double right_command_value = right_motor_handle->command;

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
   return *pointer();
}

auto motor_controls::MotorController::pointer() -> std::shared_ptr<MotorController>
{
   if (g_controller == nullptr) {
      g_controller = std::make_shared<MotorController>();
   }

   return g_controller;
}
