add_executable(motor_controls motor_controls.cpp Motor.cpp)
target_link_libraries(motor_controls PRIVATE gpio_bridge::gpio)

target_include_directories(motor_controls
        PRIVATE
        WIRINGPI_INCLUDE_DIRS)

install(TARGETS motor_controls
        EXPORT export_${PROJECT_NAME}
        DESTINATION lib/${PROJECT_NAME})
