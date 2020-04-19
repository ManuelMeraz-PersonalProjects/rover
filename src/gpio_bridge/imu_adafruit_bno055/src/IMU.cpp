#include "imu/IMU.hpp"

#include <iostream>
#include <sstream>

using namespace std::chrono_literals;
namespace gpio_bridge::imu {
IMU::IMU() : m_sensor(UNIQUE_IMU_ID, ODROID_N2_I2C_ADDRESS)
{
   bool initializing = true;
   while (initializing) {
      StatusResults results;
      m_sensor.getSystemStatus(reinterpret_cast<uint8_t*>(&results.system_status),
                               reinterpret_cast<uint8_t*>(&results.self_test_results),
                               reinterpret_cast<uint8_t*>(&results.system_error));

      initializing = handle_results(results);
   }
}

auto IMU::get() -> IMU&
{
   static IMU imu;
   return imu;
}

auto IMU::data() -> const Data&
{
   return m_data;
}

auto IMU::handle_results(const StatusResults& results) -> bool
{
   bool initializing{true};
   bool error{false};

   switch (results.system_status) {
   case SystemStatus::IDLE:
      std::cout << "Status is idle. Initializing";
      break;
   case SystemStatus::ERROR:
      std::cerr << "Error. Failed to initialize. Error is: ";
      error = true;
      initializing = false;
      break;
   case SystemStatus::INITIALIZING_PERIPHERALS:
      std::cout << "Initializing peripherals.";
      break;
   case SystemStatus::INITIALIZATION:
      std::cout << "Initializing.";
      break;
   case SystemStatus::SELF_TEST:
      std::cout << "Running self tests.";
      break;
   case SystemStatus::FUSION_ALGORITHM_RUNNING:
      std::cout << "Fusion algorithms are running!";
      initializing = false;
      break;
   case SystemStatus::RUNNING_WITHOUT_FUSION_ALGORITHMS:
      std::cout << "Running without fusion algorithms!";
      initializing = false;
      break;
   default:
      std::stringstream ss;
      ss << "Unknown system status: " << static_cast<uint16_t>(results.system_status) << std::endl;
      throw std::runtime_error(ss.str());
   }
   std::cout << std::endl;

   if (error) {
      std::stringstream ss;
      switch (results.system_error) {
      case SystemError::PERIPHERAL_INITIALIZATION_ERROR:
         ss << "Peripheral initialization error.";
         break;
      case SystemError::SYSTEM_INITIALIZATION_ERROR:
         ss << "System initialization error.";
         break;
      case SystemError::SELF_TEST_RESULT_FAILED:
         ss << "Self test result failed.";
         break;
      case SystemError::REGISTER_MAP_VALUE_OUT_OF_RANGE:
         ss << "Register map value out of range.";
         break;
      case SystemError::REGISTER_MAP_ADDRESS_OUT_OF_RANGE:
         ss << "Register map address out of range.";
         break;
      case SystemError::REGISTER_MAP_WRITE_ERROR:
         ss << "Register map write error.";
         break;
      case SystemError::BNO_LOW_POWER_MODE_NOT_AVAILABLE_FOR_SELECTED_OPERATION_MODE:
         ss << "Sensor low power mode not available for selected operation mode.";
         break;
      case SystemError::ACCELEROMETER_POWER_MODE_NOT_AVAILABLE:
         ss << "Accelerometer power mode not available.";
         break;
      case SystemError::FUSION_ALGORITHM_CONFIGURATION_ERROR:
         ss << "Fusion algorithm configuration error.";
         break;
      case SystemError::SENSOR_CONFIGURATION_ERROR:
         ss << "Sensor configuration error.";
         break;
      case SystemError::NO_ERROR:
         ss << "Error was detected but had no system error. Weird.";
         break;
      default:
         ss << "Unknown system error: " << static_cast<uint16_t>(results.system_error);
      }

      ss << std::endl;
      throw std::runtime_error(ss.str());
   }

   if (!initializing) {
      const auto pass_or_fail = [](uint8_t value) {
         std::string str{};
         if (value == 1) {
            str = "PASS!";
         } else {
            str = "Fail!";
         }

         return str;
      };

      std::cout << "Self Test Results:" << std::endl;
      std::cout << "Accelerometer: " << pass_or_fail(results.self_test_results.accelerometer) << std::endl;
      std::cout << "Magnetometer: " << pass_or_fail(results.self_test_results.magnetometer) << std::endl;
      std::cout << "Gyroscope: " << pass_or_fail(results.self_test_results.gyroscope) << std::endl;
      std::cout << "MCU: " << pass_or_fail(results.self_test_results.MCU) << std::endl;
   }

   return initializing;
}

} // namespace gpio_bridge::imu
