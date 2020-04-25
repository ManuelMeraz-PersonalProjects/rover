#include <geometry_msgs/msg/twist_stamped.hpp>
#include <rclcpp/rclcpp.hpp>

constexpr auto QUALITY_OF_SERVICE = 10;
using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class CommandVelocityPublisher : public rclcpp::Node
{
 public:
   CommandVelocityPublisher() : Node("command_velocity_publisher")
   {
      m_command_velocity_publisher =
         this->create_publisher<geometry_msgs::msg::TwistStamped>("/diff_drive_controller/cmd_vel", QUALITY_OF_SERVICE);

      m_timer = this->create_wall_timer(100ms, [this] {
         geometry_msgs::msg::TwistStamped message;
         message.header.stamp = get_clock()->now();
         message.header.frame_id = "base_link";
         message.twist.linear.x = m_linear_command;
         message.twist.linear.y = 0;
         message.twist.linear.z = 0;

         message.twist.angular.x = 0;
         message.twist.angular.y = 0;
         message.twist.angular.z = m_angular_command;

         if (m_forward) {
            m_linear_command += DELTA;
            m_angular_command += DELTA;
         } else {
            m_linear_command -= DELTA;
            m_angular_command -= DELTA;
         }

         if (std::abs(m_linear_command) > 1) {
            m_forward = !m_forward;
         }

         const auto& logger = get_logger();
         RCLCPP_INFO(logger, "Publishing out a linear command of: %f", m_linear_command);
         RCLCPP_INFO(logger, "Publishing out a angular command of: %f", m_angular_command);
         m_command_velocity_publisher->publish(message);
      });
   }

 private:
   static constexpr double DELTA = 0.05;

   bool m_forward{true};
   double m_linear_command{0.0};
   double m_angular_command{0.0};
   rclcpp::TimerBase::SharedPtr m_timer{};
   rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr m_command_velocity_publisher{};
};

int main(int argc, char* argv[])
{
   rclcpp::init(argc, argv);
   rclcpp::spin(std::make_shared<CommandVelocityPublisher>());
   rclcpp::shutdown();
   return 0;
}
