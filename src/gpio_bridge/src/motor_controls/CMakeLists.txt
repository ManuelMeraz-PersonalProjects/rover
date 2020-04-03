add_executable(motor_controls motor_controls.cpp Motor.cpp MotorController.cpp)
target_link_libraries(motor_controls PRIVATE gpio_bridge::gpio)
