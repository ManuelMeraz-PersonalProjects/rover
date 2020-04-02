#include "Motor.hpp"

motor_controls::Motor::Motor(const gpio::digital::Pin& dir_pin, const gpio::pwm::Pin& pwm_pin) :
   m_direction(Direction::FORWARD), m_duty_cycle(0), m_dir_pin(dir_pin), m_pwm_pin(pwm_pin)
{
   stop();
}

motor_controls::Motor::~Motor()
{
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

void motor_controls::Motor::actuate(Direction direction, uint8_t duty_cycle)
{
   if (m_direction == Direction::FORWARD) {
      m_dir_pin.write(gpio::digital::Write::LOW);
   } else if (m_direction == Direction::REVERSE) {
      m_dir_pin.write(gpio::digital::Write::HIGH);
   }

   if (direction != m_direction) {
      stop();
      m_direction = direction;

      if (m_direction == Direction::FORWARD) {
         m_dir_pin.write(gpio::digital::Write::LOW);
      } else if (m_direction == Direction::REVERSE) {
         m_dir_pin.write(gpio::digital::Write::HIGH);
      }
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
}
