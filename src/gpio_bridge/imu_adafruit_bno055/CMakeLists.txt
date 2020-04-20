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

find_package(rclcpp REQUIRED)
add_library(rclcpp INTERFACE)
target_link_libraries(rclcpp INTERFACE ${rclcpp_LIBRARIES})
target_include_directories(rclcpp INTERFACE ${rclcpp_INCLUDE_DIRS})

find_package(sensor_msgs REQUIRED)
add_library(sensor_msgs INTERFACE)
target_link_libraries(sensor_msgs INTERFACE ${sensor_msgs_LIBRARIES})
target_include_directories(sensor_msgs INTERFACE ${sensor_msgs_INCLUDE_DIRS})

find_package(geometry_msgs REQUIRED)
add_library(geometry_msgs INTERFACE)
target_link_libraries(geometry_msgs INTERFACE ${geometry_msgs_LIBRARIES})
target_include_directories(geometry_msgs INTERFACE ${geometry_msgs_INCLUDE_DIRS})

find_package(std_msgs REQUIRED)
add_library(std_msgs INTERFACE)
target_link_libraries(std_msgs INTERFACE ${std_msgs_LIBRARIES})
target_include_directories(std_msgs INTERFACE ${std_msgs_INCLUDE_DIRS})

add_library(imu_adafruit_bno055
        SHARED
        src/Sensor.cpp
        src/Data.cpp
        src/Calibration.cpp
        src/RunningStatistic.cpp
        src/Statistics.cpp)

target_link_libraries(imu_adafruit_bno055
        PUBLIC
        OdroidAdafruitBNO055)

target_include_directories(imu_adafruit_bno055
        PUBLIC
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)

add_executable(imu_bno055_publisher src/imu_bno055_publisher.cpp)
target_link_libraries(imu_bno055_publisher PRIVATE imu_adafruit_bno055 rclcpp sensor_msgs geometry_msgs std_msgs)

install(TARGETS
        imu_bno055_publisher
        DESTINATION lib/${PROJECT_NAME})

install(
        DIRECTORY launch
        DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()

    list(APPEND tests
            sensorapi
            read_all_data
            rawdata
            position)

    foreach(test IN LISTS tests)
        add_executable(${test} test/${test}.cpp)
        target_link_libraries(${test} PRIVATE OdroidAdafruitBNO055)
    endforeach()

    add_executable(test_imu test/test_imu.cpp)
    target_link_libraries(test_imu PRIVATE imu_adafruit_bno055)

endif()

ament_package()
