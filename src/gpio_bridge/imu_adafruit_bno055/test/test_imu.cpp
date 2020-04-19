#include <imu/IMU.hpp>

int main()
{
   auto& imu = gpio_bridge::imu::IMU::get();
   return 0;
}