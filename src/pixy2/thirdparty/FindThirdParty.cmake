set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/modules)

find_package(wiringPi REQUIRED)
find_package(Threads REQUIRED)
find_library(rt rt REQUIRED)
find_library(crypt crypt REQUIRED)

