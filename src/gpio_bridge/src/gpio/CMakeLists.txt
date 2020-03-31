add_library(gpio
        gpio.hpp
        gpio.cpp
        PwmPin.hpp
        PwmPin.cpp Pin.cpp Pin.hpp)

add_library(gpio_bridge::gpio ALIAS gpio)

target_link_libraries(gpio
        PUBLIC
        wiringPi
        Threads::Threads
        rt
        crypt)

target_include_directories(gpio
        PUBLIC
        WIRINGPI_INCLUDE_DIRS)
