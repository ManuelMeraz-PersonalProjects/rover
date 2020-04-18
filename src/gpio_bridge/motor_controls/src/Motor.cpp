#include "motor_controls/Motor.hpp"

#include <iostream>
#include <odroid/gpio.hpp>

namespace {
constexpr uint8_t CLOCK_HZ{128};
constexpr int RANGE{100};
} // namespace

motor_controls::Motor::Motor(std::string name, gpio::digital::Pin& dir_pin, gpio::pwm::Pin& pwm_pin) :
   m_direction(Direction::FORWARD),
   m_duty_cycle(0),
   m_dir_pin(dir_pin),
   m_pwm_pin(pwm_pin),
   m_name(std::move(name)),
   m_handle(std::make_shared<MotorHandle>())
{
   m_handle->joint_state_handle = hardware_interface::JointStateHandle(
      m_name, &m_handle->position, &m_handle->velocity, &m_handle->effort);
   m_handle->joint_command_handle =
      hardware_interface::JointCommandHandle(m_name, &m_handle->command);
   m_dir_pin.write(gpio::digital::IO::LOW);
   stop();
}

auto motor_controls::Motor::direction() const -> motor_controls::Direction
{
   return m_direction;
}
auto motor_controls::Motor::duty_cycle() const -> uint8_t
{
   return m_duty_cycle;
}

void motor_controls::Motor::actuate(Direction direction,
                                    uint8_t duty_cycle,
                                    std::optional<std::chrono::milliseconds> time)
{
   if (direction != m_direction) {
      stop();

      m_pwm_pin.mode(gpio::pwm::Mode::OUTPUT);
      gpio::pwm::clock(CLOCK_HZ);
      gpio::pwm::range(RANGE);

      m_direction = direction;
      if (m_direction == Direction::FORWARD) {
         m_dir_pin.write(gpio::digital::IO::LOW);
      } else if (m_direction == Direction::REVERSE) {
         m_dir_pin.write(gpio::digital::IO::HIGH);
      }
   } else {
      m_pwm_pin.mode(gpio::pwm::Mode::OUTPUT);
      gpio::pwm::clock(CLOCK_HZ);
      gpio::pwm::range(RANGE);
   }

   if (duty_cycle > m_duty_cycle) {
      while (m_duty_cycle < duty_cycle) {
         if (duty_cycle - m_duty_cycle < DUTY_CYCLE_DELTA) {
            m_duty_cycle = duty_cycle;
            m_pwm_pin.duty_cycle(m_duty_cycle);
            break;
         }

         m_duty_cycle += DUTY_CYCLE_DELTA;
         m_pwm_pin.duty_cycle(m_duty_cycle);
         gpio::sleep(DELTA_SLEEP_TIME);
      }
   } else if (duty_cycle < m_duty_cycle) {
      while (m_duty_cycle > duty_cycle) {
         if (m_duty_cycle - duty_cycle < DUTY_CYCLE_DELTA) {
            m_duty_cycle = duty_cycle;
            m_pwm_pin.duty_cycle(m_duty_cycle);
            break;
         }

         m_duty_cycle -= DUTY_CYCLE_DELTA;
         m_pwm_pin.duty_cycle(m_duty_cycle);
         gpio::sleep(DELTA_SLEEP_TIME);
      }
   }

   if (time) {
      gpio::sleep(*time);
      stop();
   } else {
      gpio::sleep(DELTA_SLEEP_TIME);
   }
}

void motor_controls::Motor::stop()
{
   while (m_duty_cycle > 0) {
      if (m_duty_cycle < DUTY_CYCLE_DELTA) {
         m_pwm_pin.duty_cycle(0);
         break;
      }

      m_duty_cycle -= DUTY_CYCLE_DELTA;
      m_pwm_pin.duty_cycle(m_duty_cycle);
      gpio::sleep(DELTA_SLEEP_TIME);
   }

   m_pwm_pin.mode(gpio::pwm::Mode::OFF);
}

auto motor_controls::Motor::handle() const -> motor_controls::MotorHandle::sPtr
{
   return m_handle;
}
