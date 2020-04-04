set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/modules)

find_package(LibUSB REQUIRED)
add_Library(usb1 STATIC IMPORTED)
set_property(TARGET usb1 PROPERTY IMPORTED_LOCATION ${LIBUSB_1_LIBRARIES})
target_include_directories(usb1 INTERFACE ${LIBUSB_1_INCLUDE_DIRS})

find_package(Pixy2 REQUIRED)
add_library(pixy2 STATIC IMPORTED)
set_property(TARGET pixy2 PROPERTY IMPORTED_LOCATION ${LIBPIXY2_LIBRARIES})
target_include_directories(pixy2 INTERFACE ${LIBPIXY2_INCLUDE_DIRS})
target_link_libraries(pixy2 INTERFACE usb1)

