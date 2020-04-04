add_library(motor_controls SHARED Motor.cpp MotorController.cpp motor_controls.hpp)
add_library(gpio_bridge::motor_controls ALIAS motor_controls)
target_link_libraries(motor_controls PRIVATE ${GPIO_LIBARIES})
target_include_directories(motor_controls PRIVATE ${GPIO_INCLUDE_DIRS})

target_include_directories(motor_controls
        PUBLIC
        ${PROJECT_SOURCE_DIR}/src/)
