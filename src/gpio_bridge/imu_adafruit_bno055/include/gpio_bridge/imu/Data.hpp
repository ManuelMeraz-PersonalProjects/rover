#ifndef IMU_ADAFRUIT_BNO055_DATA_HPP
#define IMU_ADAFRUIT_BNO055_DATA_HPP

#include <Adafruit_Sensor.h>
#include <utility/vector.h>

namespace gpio_bridge::imu {
struct Data
{
   ::imu::Vector<3> accelerometer{0, 0, 0}; // VECTOR_ACCELEROMETER - m/s^2
   ::imu::Vector<3> magnetometer{0, 0, 0};  // VECTOR_MAGNETOMETER  - uT
   ::imu::Vector<3> gyroscope{0, 0, 0};     // VECTOR_GYROSCOPE     - rad/s
   ::imu::Vector<3> euler{};                // VECTOR_EULER         - degrees
   ::imu::Vector<3> linear_acceleration{};  // VECTOR_LINEARACCEL   - m/s^2
   ::imu::Vector<3> gravity{};              // VECTOR_GRAVITY       - m/s^2

   ::imu::Quaternion quaternion{};
   uint8_t temperature{}; // Celsius

   friend auto operator<<(std::ostream& os, const Data& data) -> std::ostream&;
};

auto operator<<(std::ostream& os, const Data& data) -> std::ostream&
{
   os << "Accelerometer       {" << data.accelerometer.x() << ", " << data.accelerometer.y() << " ,"
      << data.accelerometer.z() << "}" << std::endl;
   os << "Magnetometer        {" << data.magnetometer.x() << ", " << data.magnetometer.y() << " ,"
      << data.magnetometer.z() << "}" << std::endl;
   os << "Gyroscope           {" << data.gyroscope.x() << ", " << data.gyroscope.y() << " ," << data.gyroscope.z()
      << "}" << std::endl;
   os << "Euler               {" << data.euler.x() << ", " << data.euler.y() << " ," << data.euler.z() << "}"
      << std::endl;
   os << "Linear Acceleration {" << data.linear_acceleration.x() << ", " << data.linear_acceleration.y() << " ,"
      << data.linear_acceleration.z() << "}" << std::endl;
   os << "Gravity             {" << data.gravity.x() << ", " << data.gravity.y() << " ," << data.gravity.z() << "}"
      << std::endl;
   os << "Quaternion          {" << data.quaternion.w() << ", " << data.quaternion.x() << ", " << data.quaternion.y()
      << " ," << data.quaternion.z() << "}" << std::endl;
   os << "Temperature         {" << static_cast<uint16_t>(data.temperature) << "}" << std::endl;
   return os;
}

} // namespace gpio_bridge::imu

#endif // IMU_ADAFRUIT_BNO055_DATA_HPP
