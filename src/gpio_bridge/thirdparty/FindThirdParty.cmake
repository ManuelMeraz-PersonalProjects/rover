set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/modules)

find_package(gpio REQUIRED)
add_library(odroid::gpio STATIC IMPORTED)
set_property(TARGET odroid::gpio PROPERTY IMPORTED_LOCATION ${GPIO_LIBRARIES})
target_include_directories(odroid::gpio INTERFACE ${GPIO_INCLUDE_DIRS})
