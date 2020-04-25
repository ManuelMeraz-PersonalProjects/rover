#include "gpio_bridge/imu/Sensor.hpp"

#include <rclcpp/rclcpp.hpp>
#include <sstream>

using namespace std::chrono_literals;
namespace gpio_bridge::imu {
Sensor::Sensor() : m_sensor(UNIQUE_IMU_ID, ODROID_N2_I2C_ADDRESS)
{
   bool initializing = true;
   while (initializing) {
      StatusResults results;
      m_sensor.getSystemStatus(reinterpret_cast<uint8_t*>(&results.system_status),
                               reinterpret_cast<uint8_t*>(&results.self_test_results),
                               reinterpret_cast<uint8_t*>(&results.system_error));

      initializing = handle_results(results);
      if (!initializing) {
         gpio::sleep(IMU_SAMPLE_RATE);
      }
   }

   m_sensor.setExtCrystalUse(true);
}

auto Sensor::get() -> Sensor&
{
   static Sensor imu;
   return imu;
}

auto Sensor::data() -> const Data&
{
   m_data.accelerometer = m_sensor.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
   m_data.magnetometer = m_sensor.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);
   m_data.gyroscope = m_sensor.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
   m_data.euler = m_sensor.getVector(Adafruit_BNO055::VECTOR_EULER);
   m_data.linear_acceleration = m_sensor.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
   m_data.gravity = m_sensor.getVector(Adafruit_BNO055::VECTOR_GRAVITY);

   m_data.quaternion = m_sensor.getQuat();
   m_data.temperature = m_sensor.getTemp();
   return m_data;
}

auto Sensor::handle_results(const StatusResults& results) -> bool
{
   const auto& logger = rclcpp::get_logger("IMU Status Results");
   bool initializing{true};
   bool error{false};

   switch (results.system_status) {
   case SystemStatus::IDLE:
      RCLCPP_INFO(logger, "Status is idle. Initializing");
      break;
   case SystemStatus::ERROR:
      RCLCPP_ERROR(logger, "Failed to initialize. Error is: ");
      error = true;
      initializing = false;
      break;
   case SystemStatus::INITIALIZING_PERIPHERALS:
      RCLCPP_INFO(logger, "Initializing peripherals.");
      break;
   case SystemStatus::INITIALIZATION:
      RCLCPP_INFO(logger, "Initializing.");
      break;
   case SystemStatus::SELF_TEST:
      RCLCPP_INFO(logger, "Running self tests.");
      break;
   case SystemStatus::FUSION_ALGORITHM_RUNNING:
      RCLCPP_INFO(logger, "Fusion algorithms are running!");
      initializing = false;
      break;
   case SystemStatus::RUNNING_WITHOUT_FUSION_ALGORITHMS:
      RCLCPP_INFO(logger, "Running without fusion algorithms!");
      initializing = false;
      break;
   default:
      std::stringstream ss;
      ss << "Unknown system status: " << static_cast<uint16_t>(results.system_status) << std::endl;
      throw std::runtime_error(ss.str());
   }

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

      std::stringstream self_test_results;
      self_test_results << "Self Test Results:" << std::endl;
      self_test_results << "Accelerometer: " << pass_or_fail(results.self_test_results.accelerometer) << std::endl;
      self_test_results << "Magnetometer: " << pass_or_fail(results.self_test_results.magnetometer) << std::endl;
      self_test_results << "Gyroscope: " << pass_or_fail(results.self_test_results.gyroscope) << std::endl;
      self_test_results << "MCU: " << pass_or_fail(results.self_test_results.MCU) << std::endl;
      RCLCPP_INFO(logger, self_test_results.str());
   }

   return initializing;
}

auto Sensor::calibration_status() -> const Calibration&
{
   /* Get the four calibration values (0..3) */
   /* Any sensor data reporting 0 should be ignored, */
   /* 3 means 'fully calibrated" */
   m_sensor.getCalibration(&m_calibration_status.system,
                           &m_calibration_status.gyroscope,
                           &m_calibration_status.accelerometer,
                           &m_calibration_status.magnetometer);

   return m_calibration_status;
}
} // namespace gpio_bridge::imu
