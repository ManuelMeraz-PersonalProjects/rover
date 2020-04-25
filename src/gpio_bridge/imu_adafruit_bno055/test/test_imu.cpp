#include <gpio_bridge/imu/Sensor.hpp>
#include <rclcpp/rclcpp.hpp>

using namespace std::chrono_literals;
auto main() -> int
{
   namespace imu = gpio_bridge::imu;
   const auto& logger = rclcpp::get_logger("Test IMU");
   auto& sensor = imu::Sensor::get();

   while (true) {
      std::stringstream imu_str;
      imu_str << std::endl << sensor.calibration_status() << std::endl << sensor.data() << std::endl;
      RCLCPP_INFO(logger, imu_str.str());
      gpio::sleep(gpio_bridge::imu::IMU_SAMPLE_RATE);
   }
   return 0;
}