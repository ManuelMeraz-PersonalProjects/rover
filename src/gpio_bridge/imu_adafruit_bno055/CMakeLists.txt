cmake_minimum_required(VERSION 3.10)
project(imu_adafruit_bno055)

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
find_package(OdroidAdafruitBNO055 REQUIRED)

#add_library(imu_adafruit_bno055
        #SHARED
        #src/Motor.cpp
        #src/MotorController.cpp)

#target_link_libraries(imu_adafruit_bno055
        #PUBLIC
        #OdroidGPIO
        #Threads::Threads
        #controller_interface
        #rclcpp)

#target_include_directories(imu_adafruit_bno055
        #PUBLIC
        #$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
        #$<INSTALL_INTERFACE:include>)

#ament_export_targets(export_imu_adafruit_bno055 HAS_LIBRARY_TARGET)

#install(DIRECTORY include/
        #DESTINATION include)

# need to export rclcpp and controller_interface for some reason
#install(TARGETS rclcpp
        #EXPORT export_imu_adafruit_bno055)

#install(TARGETS imu_adafruit_bno055
        #EXPORT export_imu_adafruit_bno055
        #LIBRARY DESTINATION lib
        #ARCHIVE DESTINATION lib
        #RUNTIME DESTINATION bin
        #INCLUDES DESTINATION include)

if(BUILD_TESTING)
	find_package(ament_lint_auto REQUIRED)
	ament_lint_auto_find_test_dependencies()
	list(APPEND examples
		sensorapi
		restore_offsets
		read_all_data
		rawdata
		position
		bunny)

	foreach(example IN LISTS examples)
		add_executable(${example} examples/${example}/${example}.cpp)
		target_link_libraries(${example} PRIVATE OdroidAdafruitBNO055)
	endforeach()

    #find_package(controller_manager REQUIRED)
    #add_library(controller_manager INTERFACE)
    #target_link_libraries(controller_manager INTERFACE ${controller_manager_LIBRARIES})
    #target_include_directories(controller_manager INTERFACE ${controller_manager_INCLUDE_DIRS})

    #find_package(diff_drive_controller REQUIRED)
    #add_library(diff_drive_controller INTERFACE)
    #target_link_libraries(diff_drive_controller INTERFACE ${diff_drive_controller_LIBRARIES})
    #target_include_directories(diff_drive_controller INTERFACE ${diff_drive_controller_INCLUDE_DIRS})

    #find_package(joint_state_controller REQUIRED)
    #add_library(joint_state_controller INTERFACE)
    #target_link_libraries(joint_state_controller INTERFACE ${joint_state_controller_LIBRARIES})
    #target_include_directories(joint_state_controller INTERFACE ${joint_state_controller_INCLUDE_DIRS})

    #add_executable(test_imu_adafruit_bno055 test/test_imu_adafruit_bno055.cpp)
    #add_test(NAME test_imu_adafruit_bno055 COMMAND test_imu_adafruit_bno055)
    #target_link_libraries(test_imu_adafruit_bno055 PRIVATE imu_adafruit_bno055)

    #add_executable(test_hardware_interface test/test_hardware_interface.cpp)
    #add_test(NAME test_hardware_interface COMMAND test_hardware_interface)
    #target_link_libraries(test_hardware_interface PRIVATE imu_adafruit_bno055 diff_drive_controller controller_manager joint_state_controller)
    #ament_export_dependencies(ros_controllers)
    #ament_export_dependencies(controller_manager)

    #install(TARGETS controller_manager
        #EXPORT export_imu_adafruit_bno055)

endif()


ament_package()
