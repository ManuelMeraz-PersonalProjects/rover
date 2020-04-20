#include <gpio_bridge/imu/Sensor.hpp>

using namespace std::chrono_literals;
auto main() -> int
{
   namespace imu = gpio_bridge::imu;
   auto& sensor = imu::Sensor::get();

   while (true) {
      std::cout << sensor.calibration_status() << std::endl;
      std::cout << sensor.data() << std::endl;
      std::cout << sensor.statistics() << std::endl;
      gpio::sleep(gpio_bridge::imu::IMU_SAMPLE_RATE);
   }
   return 0;
}