//
// Created by manny on 4/2/20.
//

#ifndef GPIO_BRIDGE_MOTORCONTROLLER_HPP
#define GPIO_BRIDGE_MOTORCONTROLLER_HPP

/*
 * Motor Controller | WiringPi | Physical Pin | Wire Color
 * -------------------------------------------------------
 * dir1             |21        | 31           | Blue
 * pwm1             |23        | 35           | Green
 * dir2             |22        | 33           | Yellow
 * pwm2             |24        | 37           | Red
 * gnd              | -        | 39           | Black
 */
static constexpr int PWM1 = 23; // WiringPi Pin 23; Physical Pin 35;  Green Wire
static constexpr int DIR1 = 21; // WiringPi Pin 21; Physical Pin

static constexpr int PWM2 = 24;
static constexpr int DIR2 = 22;
class MotorController
{
};

#endif // GPIO_BRIDGE_MOTORCONTROLLER_HPP
