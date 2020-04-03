cmake_minimum_required(VERSION 3.1)
project(libpixyusb2)
message("Building ${PROJECT_NAME}: pixy2/src/host/libpixyusb2")

find_package(LibUSB REQUIRED)

file(GLOB_RECURSE ${PROJECT_NAME}_SRCS
  "src/*.cpp"
)
file(GLOB_RECURSE ${PROJECT_NAME}_HEADERS
  "include/*.h"
  "../../common/inc/*" # both .h | .hpp
  "../arduino/libraries/Pixy2/*.h"
)
message("Heads:")
message(${${PROJECT_NAME}_HEADERS})
file(GLOB_RECURSE ${PROJECT_NAME}_INCLUDE_DIRS
  LIST_DIRECTORIES true
  "include"
  "../../common/inc"
  "../arduino/libraries/Pixy2"
)

add_library(${PROJECT_NAME}_chirpdep
  "../../common/src/chirp.cpp"
)
target_include_directories(
  ${PROJECT_NAME}_chirpdep PUBLIC
  "../../common/inc"
)

# message("${${PROJECT_NAME}_HEADERS}")
# message("${${PROJECT_NAME}_INCLUDE_DIRS}")

add_library(${PROJECT_NAME}
  ${${PROJECT_NAME}_SRCS}
  ${${PROJECT_NAME}_HEADERS}
)
target_compile_definitions(${PROJECT_NAME} PRIVATE CORE_M4 )
target_link_libraries(${PROJECT_NAME}
  ${PROJECT_NAME}_chirpdep
  ${LIBUSB_1_LIBRARIES}
)
target_include_directories(
  ${PROJECT_NAME} PUBLIC ${${PROJECT_NAME}_INCLUDE_DIRS} ${LIBUSB_1_INCLUDE_DIRS}
)
set_property(TARGET ${PROJECT_NAME} PROPERTY CXX_STANDARD 11)
