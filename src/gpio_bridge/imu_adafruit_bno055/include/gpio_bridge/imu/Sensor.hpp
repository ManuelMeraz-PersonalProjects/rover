#ifndef GPIO_BRIDGE_IMU_HPP
#define GPIO_BRIDGE_IMU_HPP

#include "Data.hpp"
#include "calibration.hpp"

#include <Adafruit_BNO055.h>
#include <chrono>
#include <filesystem>

namespace gpio_bridge::imu {
constexpr std::chrono::milliseconds IMU_SAMPLE_RATE{100};

class Sensor
{
 public:
   Sensor(const Sensor&) = delete;
   Sensor(Sensor&&) = delete;
   auto operator=(const Sensor&) -> Sensor& = delete;
   auto operator=(Sensor &&) -> Sensor& = delete;

   [[nodiscard]] static auto get() -> Sensor&;
   [[nodiscard]] auto data() -> const Data&;
   [[nodiscard]] auto calibration_status() -> const Calibration&;
   [[nodiscard]] auto fully_calibrated() -> bool;
   [[nodiscard]] auto calibration_offets() -> adafruit_bno055_offsets_t;
   auto print_calibration_offets() -> void;

   auto set_calibraton_offsets(const adafruit_bno055_offsets_t& calibration_data) -> void;
   auto load_calibration_data(const std::filesystem::path& calibration_data_path) -> void;

 private:
   enum class SystemStatus : uint8_t {
      IDLE = 0,
      ERROR,
      INITIALIZING_PERIPHERALS,
      INITIALIZATION,
      SELF_TEST,
      FUSION_ALGORITHM_RUNNING,
      RUNNING_WITHOUT_FUSION_ALGORITHMS
   };

   static constexpr uint8_t SELF_TEST_PASS = 0x0F;
   struct SelfTestResults
   {
      /*  1 = test passed, 0 = test failed */
      uint8_t accelerometer : 1;
      uint8_t magnetometer : 1;
      uint8_t gyroscope : 1;
      uint8_t MCU : 1;
      uint8_t : 4;
   } m_self_test_result{0, 0, 0, 0};

   enum class SystemError : uint8_t {
      NO_ERROR = 0,
      PERIPHERAL_INITIALIZATION_ERROR,
      SYSTEM_INITIALIZATION_ERROR,
      SELF_TEST_RESULT_FAILED,
      REGISTER_MAP_VALUE_OUT_OF_RANGE,
      REGISTER_MAP_ADDRESS_OUT_OF_RANGE,
      REGISTER_MAP_WRITE_ERROR,
      BNO_LOW_POWER_MODE_NOT_AVAILABLE_FOR_SELECTED_OPERATION_MODE,
      ACCELEROMETER_POWER_MODE_NOT_AVAILABLE,
      FUSION_ALGORITHM_CONFIGURATION_ERROR,
      SENSOR_CONFIGURATION_ERROR
   };

   struct StatusResults
   {
      SystemStatus system_status{};
      SelfTestResults self_test_results{};
      SystemError system_error{};
   };

   static constexpr auto ODROID_N2_I2C_ADDRESS = 0x28;
   static constexpr auto UNIQUE_IMU_ID = 1;

   Sensor();
   ~Sensor();

   static auto handle_results(const StatusResults& results) -> bool;

   Adafruit_BNO055 m_sensor;
   Data m_data{};
   Calibration m_calibration_status{};
   std::filesystem::path m_calibration_data_path{};
};
} // namespace gpio_bridge::imu
#endif // GPIO_BRIDGE_IMU_HPP
