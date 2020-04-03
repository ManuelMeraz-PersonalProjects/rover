//
// Created by manny on 4/2/20.
//

#include "MotorController.hpp"

using namespace std::chrono_literals;
namespace digital = gpio::digital;
namespace pwm = gpio::pwm;

namespace {
constexpr int DIR1_WIRING_PI_PIN = 21;
constexpr int DIR2_WIRING_PI_PIN = 22;
constexpr int PWM1_WIRING_PI_PIN = 23;
constexpr int PWM2_WIRING_PI_PIN = 24;
} // namespace

motor_controls::MotorController::MotorController()
{
   auto dir1_pin = std::make_shared<digital::Pin>(DIR1_WIRING_PI_PIN, digital::Mode::OUTPUT);
   auto dir2_pin = std::make_shared<digital::Pin>(DIR2_WIRING_PI_PIN, digital::Mode::OUTPUT);
   auto pwm1_pin = std::make_shared<pwm::Pin>(PWM1_WIRING_PI_PIN, pwm::Mode::OUTPUT);
   auto pwm2_pin = std::make_shared<pwm::Pin>(PWM2_WIRING_PI_PIN, pwm::Mode::OUTPUT);

   constexpr uint8_t CLOCK_HZ = 128;
   pwm::clock(CLOCK_HZ);

   constexpr int RANGE = 100;
   pwm::range(RANGE);

   m_left_motor = std::make_shared<motor_controls::Motor>(dir2_pin, pwm2_pin);
   m_right_motor = std::make_shared<motor_controls::Motor>(dir1_pin, pwm1_pin);
}
