cmake_minimum_required(VERSION 3.10)
project(gpio_bridge)

if (NOT CMAKE_C_STANDARD)
    set(CMAKE_C_STANDARD 99)
endif ()

if (NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif ()

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

add_subdirectory(thirdparty)

# find dependencies
find_package(Threads REQUIRED)
find_library(rt rt REQUIRED)
find_library(crypt crypt REQUIRED)

add_executable(motor_controls src/motor_controls/motor_controls.cpp)
target_link_libraries(motor_controls PRIVATE wiringPi Threads::Threads rt crypt)

target_include_directories(motor_controls
        PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
        PRIVATE
        WIRINGPI_INCLUDE_DIRS)

install(TARGETS motor_controls
        EXPORT export_${PROJECT_NAME}
        DESTINATION lib/${PROJECT_NAME})

if (BUILD_TESTING)
    enable_testing()
endif ()
