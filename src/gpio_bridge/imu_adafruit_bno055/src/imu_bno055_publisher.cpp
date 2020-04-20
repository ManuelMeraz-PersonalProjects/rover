#include "gpio_bridge/imu/Sensor.hpp"

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>

constexpr auto QUALITY_OF_SERVICE = 10;
using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class IMUBNO055Publisher : public rclcpp::Node
{
 public:
   IMUBNO055Publisher() : Node("imu_bno055_publisher")
   {
      m_imu_publisher = this->create_publisher<sensor_msgs::msg::Imu>("imu/data", QUALITY_OF_SERVICE);
      m_magnetic_field_publisher =
         this->create_publisher<sensor_msgs::msg::MagneticField>("imu/mag", QUALITY_OF_SERVICE);

      m_timer = this->create_wall_timer(gpio_bridge::imu::IMU_SAMPLE_RATE, [this] { timercallback(); });
   }

 private:
   void timercallback()
   {
      const auto& data = m_sensor.data();
      auto header = std_msgs::msg::Header();
      header.stamp = get_clock()->now();
      header.frame_id = "imu_bno055_link";

      auto imu_message = sensor_msgs::msg::Imu();
      imu_message.header = header;

      imu_message.linear_acceleration.x = data.linear_acceleration.x();
      imu_message.linear_acceleration.y = data.linear_acceleration.y();
      imu_message.linear_acceleration.z = data.linear_acceleration.z();
      imu_message.linear_acceleration_covariance = m_linear_acceleration_covariance;

      imu_message.orientation.w = data.quaternion.w();
      imu_message.orientation.x = data.quaternion.x();
      imu_message.orientation.y = data.quaternion.y();
      imu_message.orientation.z = data.quaternion.z();
      imu_message.orientation_covariance = m_orientation_covariance;

      imu_message.angular_velocity.x = data.gyroscope.x();
      imu_message.angular_velocity.y = data.gyroscope.y();
      imu_message.angular_velocity.z = data.gyroscope.z();
      imu_message.angular_velocity_covariance = m_angular_velocity_covariance;

      m_imu_publisher->publish(imu_message);

      auto magnetic_field_message = sensor_msgs::msg::MagneticField();
      magnetic_field_message.header = header;

      magnetic_field_message.magnetic_field.x = data.magnetometer.x();
      magnetic_field_message.magnetic_field.y = data.magnetometer.y();
      magnetic_field_message.magnetic_field.z = data.magnetometer.z();

      m_magnetic_field_publisher->publish(magnetic_field_message);
   }

   gpio_bridge::imu::Sensor& m_sensor{gpio_bridge::imu::Sensor::get()};
   rclcpp::TimerBase::SharedPtr m_timer{};
   rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr m_imu_publisher{};
   rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr m_magnetic_field_publisher{};

   /* +- 2.5 deg */
   std::array<double, 9> m_linear_acceleration_covariance{0.60, 0, 0, 0, 0.60, 0, 0, 0, 0.60};
   /* +- 3 deg/s */
   std::array<double, 9> m_orientation_covariance{0.002, 0, 0, 0, 0.002, 0, 0, 0, 0.002};
   /* +- 80mg */
   std::array<double, 9> m_angular_velocity_covariance{0.003, 0, 0, 0, 0.003, 0, 0, 0, 0.003};
};

int main(int argc, char* argv[])
{
   rclcpp::init(argc, argv);
   rclcpp::spin(std::make_shared<IMUBNO055Publisher>());
   rclcpp::shutdown();
   return 0;
}