#include "gpio_bridge/imu/Sensor.hpp"

#include <fstream>
#include <rclcpp/rclcpp.hpp>
#include <sstream>

using namespace std::chrono_literals;
namespace gpio_bridge::imu {
Sensor::Sensor() : m_sensor(UNIQUE_IMU_ID, ODROID_N2_I2C_ADDRESS)
{
   m_sensor.setExtCrystalUse(true);

   bool initializing = true;
   while (initializing) {
      StatusResults results;
      m_sensor.getSystemStatus(reinterpret_cast<uint8_t*>(&results.system_status),
                               reinterpret_cast<uint8_t*>(&results.self_test_results),
                               reinterpret_cast<uint8_t*>(&results.system_error));

      initializing = handle_results(results);
      if (initializing) {
         gpio::sleep(IMU_SAMPLE_RATE);
      }
   }

   const auto& logger = rclcpp::get_logger("IMU Calibration");
   if (m_sensor.isFullyCalibrated()) {
      RCLCPP_INFO(logger, "IMU is fully calibrated");
   } else {
      RCLCPP_WARN(logger, "IMU is NOT fully calibrated");
   }
}

Sensor::~Sensor()
{
   const auto& logger = rclcpp::get_logger("IMU Save Calibration");
   if (!m_sensor.isFullyCalibrated()) {
      RCLCPP_WARN(logger, "IMU is NOT fully calibrated. Will not save calibration data.");
      return;
   }

   RCLCPP_INFO(logger, "IMU is fully calibrated. Saving calibration data.");

   if (m_calibration_data_path.empty()) {
      RCLCPP_WARN(logger, "IMU calibration data path was not set!");
      RCLCPP_INFO(logger, "Saving calibration data in current directory: calibration_data.data");
      m_calibration_data_path = "calibration_data.dat";
   } else {
      RCLCPP_INFO(
         logger, "Calibration data was previously loaded. Saving calibration %s: ", m_calibration_data_path.c_str());
   }

   auto calibration_data = calibration_offets();
   print_calibration_offets(calibration_data);

   std::ofstream outfile;
   outfile.open(m_calibration_data_path.c_str(), std::ios::binary | std::ios::out);
   outfile.write(reinterpret_cast<char*>(&calibration_data), sizeof(calibration_data));
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
      const auto pass_or_fail = [](uint8_t value) -> std::string {
         std::string str{};
         if (value == 1) {
            str = "PASS!";
         } else {
            str = "Fail!";
         }

         return str;
      };

      RCLCPP_INFO(logger, "Self Test Results");
      RCLCPP_INFO(logger, "Accelerometer: %s", pass_or_fail(results.self_test_results.accelerometer).c_str());
      RCLCPP_INFO(logger, "Magnetometer: %s ", pass_or_fail(results.self_test_results.magnetometer).c_str());
      RCLCPP_INFO(logger, "Gyroscope: %s", pass_or_fail(results.self_test_results.gyroscope).c_str());
      RCLCPP_INFO(logger, "MCU: %s", pass_or_fail(results.self_test_results.MCU).c_str());
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

auto Sensor::load_calibration_data(const std::filesystem::path& calibration_data_path) -> void
{
   const auto& logger = rclcpp::get_logger("IMU Loading Calibration");
   if (!std::filesystem::exists(calibration_data_path)) {
      RCLCPP_ERROR(logger, "Must pass in a valid path.");
      throw std::invalid_argument(calibration_data_path.c_str());
   }

   if (m_sensor.isFullyCalibrated()) {
      RCLCPP_INFO(logger, "Won't load calibration data. BNO055 already fully calibrated");
      return;
   }

   // For saving the data later
   RCLCPP_INFO(logger, "Loading IMU calibration data from: %s", calibration_data_path.c_str());
   m_calibration_data_path = calibration_data_path;

   adafruit_bno055_offsets_t calibration_data{};
   std::ifstream calibration_data_file;
   calibration_data_file.open(m_calibration_data_path.c_str(), std::ios::binary | std::ios::in);
   calibration_data_file.read(reinterpret_cast<char*>(&calibration_data),
                              sizeof(calibration_data)); // reads 7 bytes into a cell that is either 2 or 4

   set_calibraton_offsets(calibration_data);
   print_calibration_offets(calibration_data);
}

auto Sensor::fully_calibrated() -> bool
{
   const auto status = calibration_status();
   return status.accelerometer > 0 and status.gyroscope > 0 and status.magnetometer > 0;
}

auto Sensor::calibration_offets() -> adafruit_bno055_offsets_t
{
   adafruit_bno055_offsets_t calibration_data;
   m_sensor.getSensorOffsets(calibration_data);
   return calibration_data;
}

auto Sensor::set_calibraton_offsets(const adafruit_bno055_offsets_t& calibration_data) -> void
{
   m_sensor.setSensorOffsets(calibration_data);
}

auto Sensor::print_calibration_offets(const adafruit_bno055_offsets_t& calibration_data) -> void
{
   const auto& logger = rclcpp::get_logger("IMU Calibration");
   RCLCPP_INFO(logger, "Calibration Offets");
   RCLCPP_INFO(logger, "Acceleration");
   RCLCPP_INFO(logger, "x: %d", calibration_data.accel_offset_x);
   RCLCPP_INFO(logger, "y: %d", calibration_data.accel_offset_y);
   RCLCPP_INFO(logger, "z: %d", calibration_data.accel_offset_z);
   RCLCPP_INFO(logger, "radius: %d", calibration_data.accel_radius);
   RCLCPP_INFO(logger, "Gyroscope");
   RCLCPP_INFO(logger, "x: %d", calibration_data.gyro_offset_x);
   RCLCPP_INFO(logger, "y: %d", calibration_data.mag_offset_y);
   RCLCPP_INFO(logger, "z: %d", calibration_data.mag_offset_z);
   RCLCPP_INFO(logger, "Magnetometer");
   RCLCPP_INFO(logger, "x: %d", calibration_data.mag_offset_x);
   RCLCPP_INFO(logger, "y: %d", calibration_data.mag_offset_y);
   RCLCPP_INFO(logger, "z: %d", calibration_data.mag_offset_z);
   RCLCPP_INFO(logger, "radius: %d", calibration_data.mag_radius);
}

} // namespace gpio_bridge::imu
