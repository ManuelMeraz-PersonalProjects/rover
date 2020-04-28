#include <csignal>
#include <gpio_bridge/imu/Sensor.hpp>
#include <rclcpp/rclcpp.hpp>

bool program_is_running = true;

void quit(int)
{
   const auto& logger = rclcpp::get_logger("Test IMU");
   RCLCPP_INFO(logger, "Quitting test!");
   program_is_running = false;
}

using namespace std::chrono_literals;
auto main() -> int
{
   signal(SIGINT, quit);
   namespace imu = gpio_bridge::imu;
   const auto& logger = rclcpp::get_logger("Test IMU");
   auto& sensor = imu::Sensor::get();

   while (program_is_running) {
      std::stringstream imu_str;
      imu_str << std::endl << sensor.calibration_status() << std::endl << sensor.data() << std::endl;
      RCLCPP_INFO(logger, imu_str.str());
      gpio::sleep(gpio_bridge::imu::IMU_SAMPLE_RATE);
   }

   return 0;
}