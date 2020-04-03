find_library(libpixy2 HINTS /usr/local/lib)
#find_path(WIRINGPI_INCLUDE_DIRS NAMES wiringPi.h REQUIRED)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(libpixy2 DEFAULT_MSG Pixy2)
