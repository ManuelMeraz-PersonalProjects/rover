cmake_minimum_required(VERSION 3.15)
project(rover)

set(CMAKE_CXX_STANDARD 20)


include_directories(cmake)

add_executable(rover main.cpp)
target_link_libraries(rover wiringPi)