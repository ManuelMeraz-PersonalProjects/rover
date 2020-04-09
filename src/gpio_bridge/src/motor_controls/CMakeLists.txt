add_library(motor_controls SHARED Motor.cpp MotorController.cpp)
add_library(gpio_bridge::motor_controls ALIAS motor_controls)
target_link_libraries(motor_controls PUBLIC libodroidgpio Threads::Threads)

ament_export_libraries(motor_controls)
ament_export_include_directories(include)
ament_export_dependencies(OdroidGPIO)
ament_export_dependencies(Threads::threads)
ament_export_interfaces(export_motor_controls HAS_LIBRARY_TARGET)

target_include_directories(motor_controls
        PUBLIC
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)

install(
        TARGETS motor_controls
        EXPORT export_motor_controls
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION bin
        INCLUDES DESTINATION include
)