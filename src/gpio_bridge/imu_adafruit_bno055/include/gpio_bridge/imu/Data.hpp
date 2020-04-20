#ifndef IMU_ADAFRUIT_BNO055_DATA_HPP
#define IMU_ADAFRUIT_BNO055_DATA_HPP

#include <Adafruit_Sensor.h>
#include <iomanip>
#include <iostream>
#include <utility/quaternion.h>
#include <utility/vector.h>

namespace gpio_bridge::imu {
struct Data
{
   ::imu::Vector<3> accelerometer{0, 0, 0};       // VECTOR_ACCELEROMETER - m/s^2
   ::imu::Vector<3> magnetometer{0, 0, 0};        // VECTOR_MAGNETOMETER  - uT
   ::imu::Vector<3> gyroscope{0, 0, 0};           // VECTOR_GYROSCOPE     - rad/s
   ::imu::Vector<3> euler{0, 0, 0};               // VECTOR_EULER         - degrees
   ::imu::Vector<3> linear_acceleration{0, 0, 0}; // VECTOR_LINEARACCEL   - m/s^2
   ::imu::Vector<3> gravity{0, 0, 0};             // VECTOR_GRAVITY       - m/s^2

   ::imu::Quaternion quaternion{0, 0, 0, 0};
   uint8_t temperature{0}; // Celsius

   friend auto operator<<(std::ostream& os, const Data& data) -> std::ostream&;
};
} // namespace gpio_bridge::imu

#endif // IMU_ADAFRUIT_BNO055_DATA_HPP
