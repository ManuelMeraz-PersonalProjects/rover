#include "gpio_bridge/imu/Calibration.hpp"

namespace gpio_bridge::imu {
auto operator<<(std::ostream& os, const Calibration& calibration) -> std::ostream&
{
   const auto calibration_level_str = [](uint8_t level) {
      if (level == 0) {
         return "Uncalibrated";
      }
      if (level < 3) {
         return "Partially Calibrated";
      }
      return "Fully Calibrated";
   };

   std::cout << "System:        " << calibration_level_str(calibration.system) << std::endl;
   std::cout << "Gyroscope:     " << calibration_level_str(calibration.gyroscope) << std::endl;
   std::cout << "Accelerometer: " << calibration_level_str(calibration.accelerometer) << std::endl;
   std::cout << "Magnetometer:  " << calibration_level_str(calibration.magnetometer) << std::endl;

   return os;
}
} // namespace gpio_bridge::imu