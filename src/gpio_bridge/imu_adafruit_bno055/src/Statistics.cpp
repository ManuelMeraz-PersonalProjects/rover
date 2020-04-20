#include "gpio_bridge/imu/Statistics.hpp"

#include <iomanip>

namespace gpio_bridge::imu {
auto operator<<(std::ostream& os, const Statistics& statistics) -> std::ostream&
{

   os << std::setprecision(3) << std::fixed;

   os << "Mean" << std::endl;
   os << "Accelerometer (m/s^2)         {" << statistics.accelerometer[0].mean() << ", "
      << statistics.accelerometer[1].mean() << " ," << statistics.accelerometer[2].mean() << "}" << std::endl;

   os << "Magnetometer (uT)             {" << statistics.magnetometer[0].mean() << ", "
      << statistics.magnetometer[1].mean() << " ," << statistics.magnetometer[2].mean() << "}" << std::endl;
   os << "Gyroscope (rad/s)             {" << statistics.gyroscope[0].mean() << ", " << statistics.gyroscope[1].mean()
      << " ," << statistics.gyroscope[2].mean() << "}" << std::endl;
   os << "Euler (deg)                   {" << statistics.euler[0].mean() << ", " << statistics.euler[1].mean() << " ,"
      << statistics.euler[2].mean() << "}" << std::endl;
   os << "Linear Acceleration (m/s^2)   {" << statistics.linear_acceleration[0].mean() << ", "
      << statistics.linear_acceleration[1].mean() << " ," << statistics.linear_acceleration[2].mean() << "}"
      << std::endl;
   os << "Gravity (m/s^2)               {" << statistics.gravity[0].mean() << ", " << statistics.gravity[1].mean()
      << " ," << statistics.gravity[2].mean() << "}" << std::endl;
   os << "Quaternion (w,x,y,z)          {" << statistics.quaternion[0].mean() << ", " << statistics.quaternion[1].mean()
      << ", " << statistics.quaternion[2].mean() << " ," << statistics.quaternion[3].mean() << "}" << std::endl;
   os << "Temperature (C)               {" << statistics.temperature.mean() << "}" << std::endl;

   os << "\nVariance" << std::endl;
   os << "Accelerometer (m/s^2)         {" << statistics.accelerometer[0].variance() << ", "
      << statistics.accelerometer[1].variance() << " ," << statistics.accelerometer[2].variance() << "}" << std::endl;

   os << "Magnetometer (uT)             {" << statistics.magnetometer[0].variance() << ", "
      << statistics.magnetometer[1].variance() << " ," << statistics.magnetometer[2].variance() << "}" << std::endl;
   os << "Gyroscope (rad/s)             {" << statistics.gyroscope[0].variance() << ", "
      << statistics.gyroscope[1].variance() << " ," << statistics.gyroscope[2].variance() << "}" << std::endl;
   os << "Euler (deg)                   {" << statistics.euler[0].variance() << ", " << statistics.euler[1].variance()
      << " ," << statistics.euler[2].variance() << "}" << std::endl;
   os << "Linear Acceleration (m/s^2)   {" << statistics.linear_acceleration[0].variance() << ", "
      << statistics.linear_acceleration[1].variance() << " ," << statistics.linear_acceleration[2].variance() << "}"
      << std::endl;
   os << "Gravity (m/s^2)               {" << statistics.gravity[0].variance() << ", "
      << statistics.gravity[1].variance() << " ," << statistics.gravity[2].variance() << "}" << std::endl;
   os << "Quaternion (w,x,y,z)          {" << statistics.quaternion[0].variance() << ", "
      << statistics.quaternion[1].variance() << ", " << statistics.quaternion[2].variance() << " ,"
      << statistics.quaternion[3].variance() << "}" << std::endl;
   os << "Temperature (C)               {" << statistics.temperature.variance() << "}" << std::endl;
   return os;
}
} // namespace gpio_bridge::imu
