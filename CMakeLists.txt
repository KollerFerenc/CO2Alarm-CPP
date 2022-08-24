cmake_minimum_required(VERSION 3.12)

set(PROJECT_NAME co2alarm)

include(pimoroni_pico_import.cmake)
include(pico_sdk_import.cmake)
include(pico_extras_import.cmake)

project(${PROJECT_NAME} C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Initialize the SDK
pico_sdk_init()

add_subdirectory(src)
