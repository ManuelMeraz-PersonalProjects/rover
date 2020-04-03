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
class MotorController
{
 public:
   MotorController();

   Motor::sPtr m_left_motor{};
   Motor::sPtr m_right_motor{};
};
} // namespace motor_controls
#endif // GPIO_BRIDGE_MOTORCONTROLLER_HPP
