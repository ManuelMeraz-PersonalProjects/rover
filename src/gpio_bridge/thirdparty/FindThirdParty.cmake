set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/modules)

find_package(OdroidGPIO REQUIRED)
add_library(libodroidgpio STATIC IMPORTED)
set_target_properties(libodroidgpio PROPERTIES IMPORTED_LOCATION ${ODROIDGPIO_LIBRARIES})
set_target_properties(libodroidgpio PROPERTIES INTERFACE_INCLUDE_DIRECTORIES ${ODROIDGPIO_INCLUDE_DIRS})
