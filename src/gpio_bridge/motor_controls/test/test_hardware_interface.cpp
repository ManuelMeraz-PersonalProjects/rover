#include <controller_manager/controller_manager.hpp>
#include <motor_controls/MotorController.hpp>
#include <rclcpp/rclcpp.hpp>
#include <ros_controllers/joint_trajectory_controller.hpp>

void spin(std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> exe)
{
   exe->spin();
}

int main(int argc, char** argv)
{
   if (!gpio::setup()) {
      std::cerr << "Setup Odroid GPIO Failed!" << std::endl;
      return 1;
   }

   rclcpp::init(argc, argv);

   // Logger
   const rclcpp::Logger logger = rclcpp::get_logger("motor_controller_logger");

   // create motor_controller instance
   auto motor_controller = motor_controls::MotorController::getPtr();

   // initialize the robot
   if (motor_controller->init() != hardware_interface::HW_RET_OK) {
      fprintf(stderr, "failed to initialized yumi hardware\n");
      return -1;
   }

   auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();

   //    start the controller manager with the robot hardware
   controller_manager::ControllerManager cm(motor_controller, executor);

   // load the joint state controller.
   // "ros_controllers" is the resource index from where to look for controllers
   // "ros_controllers::JointStateController" is the class we want to load
   // "motor_controller_joint_state_controller" is the name for the node to spawn
   cm.load_controller("ros_controllers",
                      "ros_controllers::JointStateController",
                      "motor_controller_state_controller");

   //    load the trajectory controller
   //   cm.load_controller("ros_controllers",
   //                      "ros_controllers::JointTrajectoryController",
   //                      "motor_controller_trajectory_controller");

   auto joint_controller = std::make_shared<ros_controllers::JointTrajectoryController>(
      motor_controller->get_registered_joint_names(),
      motor_controller->get_registered_write_op_names());

   cm.add_controller(joint_controller, "motor_controller_trajectory_controller");

   // we can either configure each controller individually through its services
   // or we use the controller manager to configure every loaded controller
   if (cm.configure() != controller_interface::CONTROLLER_INTERFACE_RET_SUCCESS) {
      RCLCPP_ERROR(logger, "at least one controller failed to configure");
      return 1;
   }

   // and activate all controller
   if (cm.activate() != controller_interface::CONTROLLER_INTERFACE_RET_SUCCESS) {
      RCLCPP_ERROR(logger, "at least one controller failed to activate");
      return 1;
   }

   // there is no async spinner in ROS 2, so we have to put the spin() in its own thread
   auto future_handle = std::async(std::launch::async, spin, executor);

   // main loop
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