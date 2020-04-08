cmake_minimum_required(VERSION 3.10)
project(gpio_bridge)

# set thirdparty_libraries and requirements
include(cmake/Setup.cmake)

add_subdirectory(src)

if(BUILD_TESTING)
    include(CTest)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
    add_subdirectory(tests)
endif()

include(cmake/SetupInstall.cmake)

ament_package()
