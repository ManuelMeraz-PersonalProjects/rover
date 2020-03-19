cmake_minimum_required(VERSION 3.10)
project(rover)

set(CMAKE_CXX_STANDARD 17)


include_directories(cmake)

find_package(Threads REQUIRED)
find_library(rt rt REQUIRED)
find_library(crypt crypt REQUIRED)


add_executable(rover main.cpp)
target_link_libraries(rover wiringPi Threads::Threads rt crypt)
