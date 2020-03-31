//
// Created by manny on 3/30/20.
//

#ifndef GPIO_BRIDGE_MOTOR_HPP
#define GPIO_BRIDGE_MOTOR_HPP

namespace motor_controls {

enum class Direction { FORWARD, BACKWARD };

enum class Side { LEFT, RIGHT };

class Motor
{
 public:
   Motor(Side side);

 private:
   /*
    * Motor Controller | WiringPi | Physical Pin | Wire Color
    * -------------------------------------------------------
    * dir1             |21        | 31           | Blue
    * pwm1             |23        | 35           | Green
    * dir2             |22        | 33           | Yellow
    * pwm2             |24        | 37           | Red
    * gnd              | -        | 39           | Black
    */
   static constexpr int PWM1 =
      23; // WiringPi Pin 23; Physical Pin 35;  Green Wire
   static constexpr int DIR1 = 21; // WiringPi Pin 21; Physical Pin

   static constexpr int PWM2 = 24;
   static constexpr int DIR2 = 22;

   Side m_side;
};

} // namespace motor_controls
#endif // GPIO_BRIDGE_MOTOR_HPP
