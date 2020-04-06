set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/modules)

find_package(Threads REQUIRED)
find_package(OdroidGPIO REQUIRED)
add_library(libodroidgpio INTERFACE)
target_link_libraries(libodroidgpio INTERFACE ${ODROIDGPIO_LIBRARIES})
target_include_directories(libodroidgpio INTERFACE ${ODROIDGPIO_INCLUDE_DIRS})
