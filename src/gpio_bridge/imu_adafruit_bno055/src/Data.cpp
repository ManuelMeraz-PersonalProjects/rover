#include "gpio_bridge/imu/Data.hpp"

namespace gpio_bridge::imu {
auto operator<<(std::ostream& os, const Data& data) -> std::ostream&
{
   os << std::setprecision(3) << std::fixed;

   os << "Accelerometer (m/s^2)         {" << data.accelerometer.x() << ", " << data.accelerometer.y() << " ,"
      << data.accelerometer.z() << "}" << std::endl;

   os << "Magnetometer (uT)             {" << data.magnetometer.x() << ", " << data.magnetometer.y() << " ,"
      << data.magnetometer.z() << "}" << std::endl;
   os << "Gyroscope (rad/s)             {" << data.gyroscope.x() << ", " << data.gyroscope.y() << " ,"
      << data.gyroscope.z() << "}" << std::endl;
   os << "Euler (deg)                   {" << data.euler.x() << ", " << data.euler.y() << " ," << data.euler.z() << "}"
      << std::endl;
   os << "Linear Acceleration (m/s^2)   {" << data.linear_acceleration.x() << ", " << data.linear_acceleration.y()
      << " ," << data.linear_acceleration.z() << "}" << std::endl;
   os << "Gravity (m/s^2)               {" << data.gravity.x() << ", " << data.gravity.y() << " ," << data.gravity.z()
      << "}" << std::endl;
   os << "Quaternion (w,x,y,z)          {" << data.quaternion.w() << ", " << data.quaternion.x() << ", "
      << data.quaternion.y() << " ," << data.quaternion.z() << "}" << std::endl;
   os << "Temperature (C)               {" << static_cast<uint16_t>(data.temperature) << "}" << std::endl;
   return os;
}
} // namespace gpio_bridge::imu
