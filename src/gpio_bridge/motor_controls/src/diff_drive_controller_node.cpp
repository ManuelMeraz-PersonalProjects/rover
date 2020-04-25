#include <controller_manager/controller_manager.hpp>
#include <diff_drive_controller/diff_drive_controller.hpp>
#include <joint_state_controller/joint_state_controller.hpp>
#include <motor_controls/MotorController.hpp>
#include <rclcpp/rclcpp.hpp>

using namespace std::chrono_literals;

class DiffDriveControllerNode : public rclcpp::Node
{
 public:
   DiffDriveControllerNode() : Node("diff_drive_controller_node")
   {
      const auto logger = this->get_logger();

      // initialize the robot
      if (m_motor_controller->init() != hardware_interface::HW_RET_OK) {
         throw std::runtime_error("Failed to initialize motor controller.");
      }

      RCLCPP_INFO(logger, "Initialized motor controller.");

      const auto joint_state_controller{std::make_shared<joint_state_controller::JointStateController>()};
      m_controller_manager.add_controller(joint_state_controller, "motor_state_controller");

      const auto diff_drive_controller{std::make_shared<diff_drive_controller::DiffDriveController>(
         std::vector<std::string>{"left_wheels"},
         std::vector<std::string>{"right_wheels"},
         m_motor_controller->get_registered_write_op_names())};
      m_controller_manager.add_controller(diff_drive_controller, "diff_drive_controller");

      if (m_controller_manager.configure() != controller_interface::CONTROLLER_INTERFACE_RET_SUCCESS) {
         throw std::runtime_error("At least one controller failed to configure");
      }

      RCLCPP_INFO(logger, "Configured controller manager.");

      if (m_controller_manager.activate() != controller_interface::CONTROLLER_INTERFACE_RET_SUCCESS) {
         throw std::runtime_error("At least one controller failed to activate");
      }

      RCLCPP_INFO(logger, "Activated controller manager.");

      // there is no async spinner in ROS 2, so we have to put the spin() in its own thread
      auto future_handle = std::async(
         std::launch::async,
         [](std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> exe) { exe->spin(); },
         m_executor);

      RCLCPP_INFO(logger, "Controllers now listening!");
      m_timer = this->create_wall_timer(100ms, [this] {
         const auto logger = this->get_logger();
         if (m_motor_controller->read() != hardware_interface::HW_RET_OK) {
            RCLCPP_WARN(logger, "Motor controller read failed!");
         }

         m_controller_manager.update();

         if (m_motor_controller->write() != hardware_interface::HW_RET_OK) {
            RCLCPP_WARN(logger, "Motor controller write failed!");
         }
      });
   }

   ~DiffDriveControllerNode()
   {
      m_executor->cancel();
   }

 private:
   static constexpr double DELTA = 0.05;

   gpio_bridge::motor_controls::MotorController::sPtr m_motor_controller{
      gpio_bridge::motor_controls::MotorController::pointer()};
   rclcpp::TimerBase::SharedPtr m_timer{};

   std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> m_executor{
      std::make_shared<rclcpp::executors::MultiThreadedExecutor>()};

   controller_manager::ControllerManager m_controller_manager{m_motor_controller, m_executor};
};

int main(int argc, char** argv)
{
   rclcpp::init(argc, argv);
   rclcpp::spin(std::make_shared<DiffDriveControllerNode>());
   rclcpp::shutdown();
}
