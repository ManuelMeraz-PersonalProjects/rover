#include "gpio_bridge/imu/calibration.hpp"

#include <sstream>

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

   std::stringstream calibration_results;
   os << "System:        " << calibration_level_str(calibration.system) << std::endl;
   os << "Gyroscope:     " << calibration_level_str(calibration.gyroscope) << std::endl;
   os << "Accelerometer: " << calibration_level_str(calibration.accelerometer) << std::endl;
   os << "Magnetometer:  " << calibration_level_str(calibration.magnetometer) << std::endl;

   return os;
}
} // namespace gpio_bridge::imu