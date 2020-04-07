add_library(motor_controls SHARED Motor.cpp MotorController.cpp)
add_library(gpio_bridge::motor_controls ALIAS motor_controls)
target_link_libraries(motor_controls PRIVATE libodroidgpio Threads::Threads)

target_include_directories(motor_controls
        PUBLIC
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include/>
        $<INSTALL_INTERFACE:include/GPIO_BRIDGE>)

install(TARGETS motor_controls
        EXPORT gpio-bridge-targets
        RUNTIME DESTINATION "${INSTALL_BIN_DIR}" COMPONENT bin
        LIBRARY DESTINATION "${INSTALL_LIB_DIR}" COMPONENT shlib
        PUBLIC_HEADER DESTINATION "${INSTALL_INCLUDE_DIR}"
        COMPONENT dev)
