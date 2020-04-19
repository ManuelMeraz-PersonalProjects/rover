#ifndef IMU_ADAFRUIT_BNO055_CALIBRATION_HPP
#define IMU_ADAFRUIT_BNO055_CALIBRATION_HPP

#include <Adafruit_Sensor.h>
#include <iostream>

namespace gpio_bridge::imu {
struct Calibration
{
   uint8_t system{};
   uint8_t gyroscope{};
   uint8_t accelerometer{};
   uint8_t magnetometer{};

   friend auto operator<<(std::ostream& os, const Calibration& calibration) -> std::ostream&;
};
} // namespace gpio_bridge::imu

#endif // IMU_ADAFRUIT_BNO055_CALIBRATION_HPP
