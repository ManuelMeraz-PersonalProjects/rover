#ifndef GPIO_BRIDGE_IMU_HPP
#define GPIO_BRIDGE_IMU_HPP

namespace gpio_bridge {
namespace imu {
struct Data
{
   // some data
};

class IMU
{
 public:
   IMU(const IMU&) = delete;
   IMU(IMU&&) = delete;
   auto operator=(const IMU&) -> IMU& = delete;
   auto operator=(IMU &&) -> IMU& = delete;

   auto data() -> const Data&;

 private:
   IMU();

   static auto get() -> IMU&;

   Data m_data;
};
} // namespace IMU
} // namespace gpio_bridge
#endif // GPIO_BRIDGE_IMU_HPP
