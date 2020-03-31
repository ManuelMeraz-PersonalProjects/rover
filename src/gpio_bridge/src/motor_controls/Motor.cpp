//
// Created by manny on 3/30/20.
//

/*
 * Motor Controller | WiringPi | Physical Pin | Wire Color
 * -------------------------------------------------------
 * dir1             |21        | 31           | Blue
 * pwm1             |23        | 35           | Green
 * dir2             |22        | 33           | Yellow
 * pwm2             |24        | 37           | Red
 * gnd              | -        | 39           | Black
 */
constexpr int PWM1 = 23; // WiringPi Pin 23; Physical Pin 35;  Green Wire
constexpr int DIR1 = 21; // WiringPi Pin 21; Physical Pin

constexpr int PWM2 = 24;
constexpr int DIR2 = 22;

#include "Motor.hpp"
motor_controls::Motor::Motor(motor_controls::Side side) : m_side(side) {}
