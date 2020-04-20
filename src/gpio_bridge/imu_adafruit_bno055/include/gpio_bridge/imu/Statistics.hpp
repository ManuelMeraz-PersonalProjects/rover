#ifndef IMU_ADAFRUIT_BNO055_STATISTICS_HPP
#define IMU_ADAFRUIT_BNO055_STATISTICS_HPP

#include "RunningStatistic.hpp"

#include <array>
#include <iostream>

namespace gpio_bridge::imu {
struct Statistics
{
   std::array<RunningStatistic, 3> accelerometer{};       // VECTOR_ACCELEROMETER - m/s^2
   std::array<RunningStatistic, 3> magnetometer{};        // VECTOR_MAGNETOMETER  - uT
   std::array<RunningStatistic, 3> gyroscope{};           // VECTOR_GYROSCOPE     - rad/s
   std::array<RunningStatistic, 3> euler{};               // VECTOR_EULER         - degrees
   std::array<RunningStatistic, 3> linear_acceleration{}; // VECTOR_LINEARACCEL   - m/s^2
   std::array<RunningStatistic, 3> gravity{};             // VECTOR_GRAVITY       - m/s^2

   std::array<RunningStatistic, 4> quaternion{};
   RunningStatistic temperature{}; // Celsius

   friend auto operator<<(std::ostream& os, const Statistics& statistics) -> std::ostream&;
};
} // namespace gpio_bridge::imu

#endif // IMU_ADAFRUIT_BNO055_STATISTICS_HPP
