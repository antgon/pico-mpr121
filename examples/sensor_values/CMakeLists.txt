# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

# Add pico-mpr121 directory
add_subdirectory(../../lib mpr121)

project(touch C CXX ASM)
pico_sdk_init()
add_executable(sensor_values sensor_values.c)

# This stdio configuration alows to see the Pico output when connected to a
# picoprobe. Enable usb and disable uart for standalone operation (direct USB
# connection).
pico_enable_stdio_uart(sensor_values 0)
pico_enable_stdio_usb(sensor_values 1)

# Standard and additional libraries.
target_link_libraries(sensor_values
        pico_stdlib
        hardware_i2c
        pico-mpr121
        )

pico_add_extra_outputs(sensor_values)
