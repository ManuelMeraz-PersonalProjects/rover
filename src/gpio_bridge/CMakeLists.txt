cmake_minimum_required(VERSION 3.10)
project(gpio_bridge)

# set thirdparty_libraries
include(cmake/Setup.cmake)
add_subdirectory(src)

if (BUILD_TESTING)
    include(CTest)
    add_subdirectory(tests)
endif ()
