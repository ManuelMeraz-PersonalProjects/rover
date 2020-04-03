find_library(LIBPIXY2_LIBRARIES NAMES libpixy2.a HINTS /usr/local/lib/libpixy2 REQUIRED)
find_path(LIBPIXY2_INCLUDE_DIRS NAMES libpixyusb2.h HINTS /usr/local/include/libpixy2 REQUIRED)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(Pixy2 DEFAULT_MSG LIBPIXY2_LIBRARIES LIBPIXY2_INCLUDE_DIRS)
