add_executable(motor_controls motor_controls.cpp Motor.cpp MotorController.cpp MotorController.hpp)
target_link_libraries(motor_controls PRIVATE gpio_bridge::gpio)
