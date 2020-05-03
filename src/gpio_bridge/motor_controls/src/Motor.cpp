#include "motor_controls/Motor.hpp"

#include <odroid/gpio.hpp>

gpio_bridge::motor_controls::Motor::Motor(std::string name, gpio::digital::Pin& dir_pin, gpio::pwm::Pin& pwm_pin) :
   m_direction(Direction::FORWARD),
   m_duty_cycle(0),
   m_dir_pin(dir_pin),
   m_pwm_pin(pwm_pin),
   m_name(std::move(name)),
   m_handle(std::make_shared<MotorHandle>())
{
   m_handle->joint_state_handle =
      hardware_interface::JointStateHandle(m_name, &m_handle->position, &m_handle->velocity, &m_handle->effort);
   m_handle->joint_command_handle = hardware_interface::JointCommandHandle(m_name, &m_handle->command);
   m_dir_pin.write(gpio::digital::IO::LOW);
   stop();
}

auto gpio_bridge::motor_controls::Motor::direction() const -> gpio_bridge::motor_controls::Direction
{
   return m_direction;
}
auto gpio_bridge::motor_controls::Motor::duty_cycle() const -> uint8_t
{
   return m_duty_cycle;
}

void gpio_bridge::motor_controls::Motor::actuate(Direction direction,
                                                 uint8_t duty_cycle,
                                                 std::optional<std::chrono::milliseconds> time)
{
   m_duty_cycle = duty_cycle;
   m_direction = direction;
   if (direction == Direction::FORWARD) {
      m_dir_pin.write(gpio::digital::IO::LOW);
   } else if (direction == Direction::REVERSE) {
      m_dir_pin.write(gpio::digital::IO::HIGH);
   }

   m_pwm_pin.duty_cycle(duty_cycle);

   if (time) {
      gpio::sleep(*time);
      stop();
   } else {
      gpio::sleep(DELTA_SLEEP_TIME);
   }
}

void gpio_bridge::motor_controls::Motor::stop()
{
   m_pwm_pin.duty_cycle(0);
}

auto gpio_bridge::motor_controls::Motor::handle() const -> gpio_bridge::motor_controls::MotorHandle::sPtr
{
   return m_handle;
}
