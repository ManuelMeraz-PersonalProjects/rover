#include "IMU/IMU.hpp"

namespace gpio_bridge::imu {
IMU::IMU() {}

auto IMU::get() -> IMU&
{
   static IMU sensor;
   return sensor;
}

auto IMU::data() -> const Data&
{
   return m_data;
}

} // namespace gpio_bridge::imu
