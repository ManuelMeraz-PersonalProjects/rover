//
// Created by manny on 4/2/20.
//

#include "MotorController.hpp"

#include <thread>

using namespace std::chrono_literals;
namespace digital = gpio::digital;
namespace pwm = gpio::pwm;

namespace {
// globals
motor_controls::MotorController* g_controller{nullptr};

constexpr int DIR1_WIRING_PI_PIN{21};
constexpr int DIR2_WIRING_PI_PIN{22};
constexpr int PWM1_WIRING_PI_PIN{23};
constexpr int PWM2_WIRING_PI_PIN{24};

constexpr uint8_t CLOCK_HZ{128};
constexpr int RANGE{100};
} // namespace

motor_controls::MotorController::MotorController()
{
   auto dir1_pin = std::make_unique<digital::Pin>(DIR1_WIRING_PI_PIN, digital::Mode::OUTPUT);
   auto dir2_pin = std::make_unique<digital::Pin>(DIR2_WIRING_PI_PIN, digital::Mode::OUTPUT);
   auto pwm1_pin = std::make_unique<pwm::Pin>(PWM1_WIRING_PI_PIN, pwm::Mode::OUTPUT);
   auto pwm2_pin = std::make_unique<pwm::Pin>(PWM2_WIRING_PI_PIN, pwm::Mode::OUTPUT);

   pwm::clock(CLOCK_HZ);
   pwm::range(RANGE);

   m_left_motor = std::make_unique<motor_controls::Motor>(std::move(dir2_pin), std::move(pwm2_pin));
   m_right_motor =
      std::make_unique<motor_controls::Motor>(std::move(dir1_pin), std::move(pwm1_pin));
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
   if (g_controller == nullptr) {
      g_controller = new MotorController();
   }

   return *g_controller;
}

motor_controls::MotorController::~MotorController()
{
   delete (g_controller);
}
