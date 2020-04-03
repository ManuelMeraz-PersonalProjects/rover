cmake_minimum_required(VERSION 3.10)
project(pixy2)

if (NOT CMAKE_C_STANDARD)
    set(CMAKE_C_STANDARD 99)
endif ()

if (NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif ()

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

# set thirdparty_libraries
#include(thirdparty/FindThirdParty.cmake)

if (BUILD_TESTING)
    enable_testing()
    #add_subdirectory(tests)
endif ()
