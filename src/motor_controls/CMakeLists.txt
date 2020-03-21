cmake_minimum_required(VERSION 3.10)
project(rover)

set(CMAKE_CXX_STANDARD 17)

add_subdirectory(thirdparty)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(Threads REQUIRED)
find_library(rt rt REQUIRED)
find_library(crypt crypt REQUIRED)

add_executable(rover main.cpp)
target_link_libraries(rover wiringPi Threads::Threads rt crypt)
target_include_directories(rover PRIVATE WIRINGPI_INCLUDE_DIRS)
