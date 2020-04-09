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
find_package(Threads REQUIRED)
find_package(OdroidGPIO REQUIRED)

add_library(motor_controls
        SHARED
        src/Motor.cpp
        src/MotorController.cpp)

target_link_libraries(motor_controls
        PUBLIC
        odroidgpio
        Threads::Threads)

target_include_directories(motor_controls
        PUBLIC
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)

install(DIRECTORY include/
        DESTINATION include)

install(TARGETS motor_controls
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION bin)

if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()

    add_executable(test_motor_controls test/test_motor_controls.cpp)
    add_test(NAME test_motor_controls COMMAND test_motor_controls)
    target_link_libraries(test_motor_controls PRIVATE motor_controls)
endif()

ament_package()
