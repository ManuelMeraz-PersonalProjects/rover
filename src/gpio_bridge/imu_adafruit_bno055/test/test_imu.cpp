#include <gpio_bridge/imu/Sensor.hpp>

auto main() -> int
{
   namespace imu = gpio_bridge::imu;
   auto& sensor = imu::Sensor::get();
   return 0;
}