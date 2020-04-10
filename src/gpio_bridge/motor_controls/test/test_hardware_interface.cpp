#include <controller_manager/controller_manager.hpp>
#include <motor_controls/MotorController.hpp>
#include <rclcpp/rclcpp.hpp>
#include <ros_controllers/joint_state_controller.hpp>
#include <ros_controllers/joint_trajectory_controller.hpp>

void spin(std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> exe)
{
   exe->spin();
}

int main(int argc, char** argv)
{
   rclcpp::init(argc, argv);

   const auto logger = rclcpp::get_logger("motor_controller_logger");
   const auto motor_controller = motor_controls::MotorController::pointer();

   // initialize the robot
   if (motor_controller->init() != hardware_interface::HW_RET_OK) {
      std::cerr << "failed to initialized yumi hardware\n";
      return 1;
   } else {
      RCLCPP_INFO(logger, "Initialized motor motor.");
   }

   const auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
   controller_manager::ControllerManager cm(motor_controller, executor);

   const auto joint_state_controller = std::make_shared<ros_controllers::JointStateController>();
   cm.add_controller(joint_state_controller, "motor_state_controller");

   const auto joint_trajectory_controller =
      std::make_shared<ros_controllers::JointTrajectoryController>(
         motor_controller->get_registered_joint_names(),
         motor_controller->get_registered_write_op_names());

   cm.add_controller(joint_trajectory_controller, "motor_trajectory_controller");

   if (cm.configure() != controller_interface::CONTROLLER_INTERFACE_RET_SUCCESS) {
      RCLCPP_ERROR(logger, "At least one controller failed to configure");
      return 1;
   } else {
      RCLCPP_INFO(logger, "Configured controller manager.");
   }

   if (cm.activate() != controller_interface::CONTROLLER_INTERFACE_RET_SUCCESS) {
      RCLCPP_ERROR(logger, "at least one controller failed to activate");
      return 1;
   } else {
      RCLCPP_INFO(logger, "Activated controller manager.");
   }

   // there is no async spinner in ROS 2, so we have to put the spin() in its own thread
   auto future_handle = std::async(std::launch::async, spin, executor);

   RCLCPP_INFO(logger, "Controllers now listening!");
   hardware_interface::hardware_interface_ret_t ret;
   rclcpp::Rate rate(10);
   while (rclcpp::ok()) {
      ret = motor_controller->read();
      if (ret != hardware_interface::HW_RET_OK) {
         fprintf(stderr, "read failed!\n");
      }

      cm.update();

      ret = motor_controller->write();
      if (ret != hardware_interface::HW_RET_OK) {
         fprintf(stderr, "write failed!\n");
      }

      rate.sleep();
   }

   executor->cancel();
}