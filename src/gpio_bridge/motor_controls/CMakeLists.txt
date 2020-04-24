cmake_minimum_required(VERSION 3.10)
project(motor_controls)

# creates compile_commands.json database for linters
if(NOT CMAKE_EXPORT_COMPILE_COMMANDS)
    set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
endif()

# Only do these if this is the main project, and not if it is included through add_subdirectory
if(NOT CMAKE_C_STANDARD)
    set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Let's ensure -std=c++xx instead of -std=g++xx
set(CMAKE_CXX_EXTENSIONS OFF)

# Let's nicely support folders in IDE's
set_property(GLOBAL PROPERTY USE_FOLDERS ON)

find_package(ament_cmake REQUIRED)
find_package(OdroidGPIO REQUIRED)

find_package(controller_interface REQUIRED)
add_library(controller_interface INTERFACE)
target_link_libraries(controller_interface INTERFACE ${controller_interface_LIBRARIES})
target_include_directories(controller_interface INTERFACE ${controller_interface_INCLUDE_DIRS})

find_package(rclcpp REQUIRED)
add_library(rclcpp INTERFACE)
target_link_libraries(rclcpp INTERFACE ${rclcpp_LIBRARIES})
target_include_directories(rclcpp INTERFACE ${rclcpp_INCLUDE_DIRS})

find_package(geometry_msgs REQUIRED)
add_library(geometry_msgs INTERFACE)
target_link_libraries(geometry_msgs INTERFACE ${geometry_msgs_LIBRARIES})
target_include_directories(geometry_msgs INTERFACE ${geometry_msgs_INCLUDE_DIRS})

find_package(std_msgs REQUIRED)
add_library(std_msgs INTERFACE)
target_link_libraries(std_msgs INTERFACE ${std_msgs_LIBRARIES})
target_include_directories(std_msgs INTERFACE ${std_msgs_INCLUDE_DIRS})

find_package(controller_manager REQUIRED)
add_library(controller_manager INTERFACE)
target_link_libraries(controller_manager INTERFACE ${controller_manager_LIBRARIES})
target_include_directories(controller_manager INTERFACE ${controller_manager_INCLUDE_DIRS})

find_package(diff_drive_controller REQUIRED)
add_library(diff_drive_controller INTERFACE)
target_link_libraries(diff_drive_controller INTERFACE ${diff_drive_controller_LIBRARIES})
target_include_directories(diff_drive_controller INTERFACE ${diff_drive_controller_INCLUDE_DIRS})

find_package(joint_state_controller REQUIRED)
add_library(joint_state_controller INTERFACE)
target_link_libraries(joint_state_controller INTERFACE ${joint_state_controller_LIBRARIES})
target_include_directories(joint_state_controller INTERFACE ${joint_state_controller_INCLUDE_DIRS})

add_library(motor_controls
        SHARED
        src/Motor.cpp
        src/MotorController.cpp)

target_link_libraries(motor_controls
        OdroidGPIO
        controller_interface
        rclcpp)

ament_target_dependencies(motor_controls
        controller_interface
        rclcpp)

target_include_directories(motor_controls
        PUBLIC
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)

add_executable(diff_drive_controller_node src/diff_drive_controller_node.cpp)
target_link_libraries(diff_drive_controller_node motor_controls diff_drive_controller controller_manager joint_state_controller)
ament_target_dependencies(diff_drive_controller_node diff_drive_controller controller_manager joint_state_controller)

ament_export_targets(export_motor_controls)
ament_export_dependencies(rclcpp)
ament_export_dependencies(controller_manager)
ament_export_dependencies(controller_interface)
ament_export_dependencies(joint_state_controller)
ament_export_dependencies(diff_drive_controller)

install(DIRECTORY include/
        DESTINATION include)

# need to export rclcpp and controller_interface for some reason
install(TARGETS rclcpp
        EXPORT export_motor_controls)

install(TARGETS controller_interface
        EXPORT export_motor_controls)

install(TARGETS diff_drive_controller_node
        EXPORT export_motor_controls
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION bin
        INCLUDES DESTINATION include)


if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()

    add_executable(test_motor_controls test/test_motor_controls.cpp)
    add_test(NAME test_motor_controls COMMAND test_motor_controls)
    target_link_libraries(test_motor_controls PRIVATE motor_controls)

    add_executable(test_hardware_interface test/test_hardware_interface.cpp)
    add_test(NAME test_hardware_interface COMMAND test_hardware_interface)
    target_link_libraries(test_hardware_interface PRIVATE motor_controls diff_drive_controller controller_manager joint_state_controller)

    add_executable(cmd_vel_publisher test/cmd_vel_publisher.cpp)
    add_test(NAME cmd_vel_publisher COMMAND cmd_vel_publisher)
    target_link_libraries(cmd_vel_publisher PRIVATE rclcpp geometry_msgs std_msgs)

    install(TARGETS controller_manager
            EXPORT export_motor_controls)

endif()


ament_package()
