#include <controller_manager/controller_manager.hpp>
#include <diff_drive_controller/diff_drive_controller.hpp>
#include <joint_state_controller/joint_state_controller.hpp>
#include <motor_controls/MotorController.hpp>
#include <rclcpp/rclcpp.hpp>
#include <utility>

using namespace std::chrono_literals;

class DiffDriveControllerNode : public rclcpp::Node
{
 public:
   DiffDriveControllerNode(gpio_bridge::motor_controls::MotorController::sPtr motor_controller,
                           std::shared_ptr<controller_manager::ControllerManager> controller_manager) :
      Node("diff_drive_controller_node"),
      m_motor_controller(std::move(motor_controller)),
      m_controller_manager(std::move(controller_manager))
   {
      const auto logger = this->get_logger();

      const auto joint_state_controller{std::make_shared<joint_state_controller::JointStateController>()};
      m_controller_manager->add_controller(joint_state_controller, "motor_state_controller");

      m_controller_manager->load_controller("diff_drive_controller", "diff_drive_controller/DiffDriveController");

      if (m_controller_manager->configure() != controller_interface::CONTROLLER_INTERFACE_RET_SUCCESS) {
         throw std::runtime_error("At least one controller failed to configure");
      }

      RCLCPP_INFO(logger, "Configured controller manager.");

      if (m_controller_manager->activate() != controller_interface::CONTROLLER_INTERFACE_RET_SUCCESS) {
         throw std::runtime_error("At least one controller failed to activate");
      }
      RCLCPP_INFO(logger, "Activated controller manager.");

      RCLCPP_INFO(logger, "Controllers now listening!");
      m_update_diff_drive_timer = this->create_wall_timer(100ms, [this]() {
         const auto logger = this->get_logger();
         if (m_motor_controller->read() != hardware_interface::HW_RET_OK) {
            RCLCPP_WARN(logger, "Motor controller read failed!");
         }

         m_controller_manager->update();

         if (m_motor_controller->write() != hardware_interface::HW_RET_OK) {
            RCLCPP_WARN(logger, "Motor controller write failed!");
         }
      });
   }

 private:
   static constexpr double DELTA = 0.05;

   gpio_bridge::motor_controls::MotorController::sPtr m_motor_controller{nullptr};
   std::shared_ptr<controller_manager::ControllerManager> m_controller_manager{nullptr};
   rclcpp::TimerBase::SharedPtr m_update_diff_drive_timer{};
};

int main(int argc, char** argv)
{
   rclcpp::init(argc, argv);
   auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
   auto motor_controller = gpio_bridge::motor_controls::MotorController::pointer();

   if (motor_controller->init() != hardware_interface::HW_RET_OK) {
      throw std::runtime_error("Failed to initialize motor controller.");
   }
   RCLCPP_INFO(rclcpp::get_logger("motor_controls"), "Initialized motor controller.");
   auto controller_manager = std::make_shared<controller_manager::ControllerManager>(motor_controller, executor);

   // add in controllers within diff drive node constructor
   auto diff_drive_node = std::make_shared<DiffDriveControllerNode>(motor_controller, controller_manager);

   executor->add_node(diff_drive_node);
   executor->spin();
   rclcpp::shutdown();
}
