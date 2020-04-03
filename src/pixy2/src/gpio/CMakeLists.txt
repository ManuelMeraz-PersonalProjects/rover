add_library(gpio
        SHARED
        digital.cpp
        gpio.cpp
        pwm.cpp)

add_library(gpio_bridge::gpio ALIAS gpio)

target_link_libraries(gpio
        PUBLIC
        wiringPi
        Threads::Threads
        rt
        crypt)

target_include_directories(gpio
        PUBLIC
        ${PROJECT_SOURCE_DIR}/src/

        PRIVATE
        WIRINGPI_INCLUDE_DIRS)
