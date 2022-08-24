set(OUTPUT_NAME co2alarm)

# Add your source files
add_executable(${OUTPUT_NAME}
    main.cpp
)

# Include required libraries
include(common/pimoroni_i2c)
include(drivers/scd4x/scd4x)

# Don't forget to link the libraries you need!
target_link_libraries(${OUTPUT_NAME}
    pico_stdlib
    hardware_gpio
    hardware_adc
    hardware_pwm
    pimoroni_i2c
    scd4x 
    hardware_sleep
    hardware_clocks
    hardware_rosc
    hardware_watchdog
)

if(${CMAKE_BUILD_TYPE} STREQUAL "Debug")
    message(STATUS ${CMAKE_BUILD_TYPE})
    add_definitions(-DDEBUG)
endif()

# create map/bin/hex file etc.
pico_add_extra_outputs(${OUTPUT_NAME})

# Commented out because with it the Pico would hang after a few RTC sleep cicles
# pico_enable_stdio_usb(${OUTPUT_NAME} 1)
# pico_enable_stdio_uart(${OUTPUT_NAME} 1)
