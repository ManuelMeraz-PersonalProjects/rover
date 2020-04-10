void spin(std::shared_ptr<rclcpp::executors::multi_threaded_executor::MultiThreadedExecutor> exe)
{
   exe->spin();
}

int main()
{
   // do all the init stuff

   // Logger
   const rclcpp::Logger logger = rclcpp::get_logger("my_robot_logger");

   // create my_robot instance
   auto my_robot = std::make_shared<MyRobot>();

   // initialize the robot
   if (my_robot->init() != hardware_interface::HW_RET_OK) {
      fprintf(stderr, "failed to initialized yumi hardware\n");
      return -1;
   }

   auto executor =
      std::make_shared<rclcpp::executors::multi_threaded_executor::MultiThreadedExecutor>();

   // start the controller manager with the robot hardware
   controller_manager::ControllerManager cm(my_robot, executor);
   // load the joint state controller.
   // "ros_controllers" is the resource index from where to look for controllers
   // "ros_controllers::JointStateController" is the class we want to load
   // "my_robot_joint_state_controller" is the name for the node to spawn
   cm.load_controller("ros_controllers",
                      "ros_controllers::JointStateController",
                      "my_robot_joint_state_controller");
   // load the trajectory controller
   cm.load_controller("ros_controllers",
                      "ros_controllers::JointTrajectoryController",
                      "my_robot_joint_trajectory_controller");

   // there is no async spinner in ROS 2, so we have to put the spin() in its own thread
   auto future_handle = std::async(std::launch::async, spin, executor);

   // we can either configure each controller individually through its services
   // or we use the controller manager to configure every loaded controller
   if (cm.configure() != controller_interface::CONTROLLER_INTERFACE_RET_SUCCESS) {
      RCLCPP_ERROR(logger, "at least one controller failed to configure")
      return -1;
   }
   // and activate all controller
   if (cm.activate() != controller_interface::CONTROLLER_INTERFACE_RET_SUCCESS) {
      RCLCPP_ERROR(logger, "at least one controller failed to activate")
      return -1;
   }

   // main loop
   hardware_interface::hardware_interface_ret_t ret;
   while (active) {
      ret = my_robot->read();
      if (ret != hardware_interface::HW_RET_OK) {
         fprintf(stderr, "read failed!\n");
      }

      cm.update();

      ret = my_robot->write();
      if (ret != hardware_interface::HW_RET_OK) {
         fprintf(stderr, "write failed!\n");
      }

      r.sleep();
   }

   executor->cancel();
}