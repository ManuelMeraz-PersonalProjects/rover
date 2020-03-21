find_library(WIRINGPI_LIBRARIES NAMES wiringPi REQUIRED)
find_path(WIRINGPI_INCLUDE_DIRS NAMES wiringPi.h REQUIRED)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(wiringPi DEFAULT_MSG WIRINGPI_LIBRARIES WIRINGPI_INCLUDE_DIRS)
