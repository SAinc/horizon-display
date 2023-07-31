cmake_minimum_required(VERSION 3.13)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Pull in SDK (must be before project)  
set(PICO_SDK_PATH "/mnt/c/Users/purpl/repos/rp2040/pico-sdk")

set(PICO_BOARD adafruit_itsybitsy_rp2040 CACHE STRING "Board type")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

if (PICO_SDK_VERSION_STRING VERSION_LESS "1.4.0")
    message(FATAL_ERROR "Raspberry Pi Pico SDK version 1.4.0 (or later) required. Your version is ${PICO_SDK_VERSION_STRING}")
endif()

project(firmware C CXX ASM)

# Initialize the SDK
pico_sdk_init()

add_executable(firmware main.cpp)

pico_set_program_name(firmware "horizon-display")
pico_set_program_version(firmware "0.1")

pico_enable_stdio_uart(firmware 0)
pico_enable_stdio_usb(firmware 1)

# Add the standard library to the build
target_link_libraries(firmware
        pico_stdlib
        hardware_uart
        pico_time
        hagl_hal
        hagl
        vesc-uart-pico)

add_subdirectory(external/vesc-uart-pico)
add_subdirectory(external/hagl)
add_subdirectory(external/hagl_pico_mipi)

target_compile_definitions(firmware PRIVATE
  HAGL_HAL_USE_SINGLE_BUFFER
#  HAGL_HAL_USE_DOUBLE_BUFFER
#  HAGL_HAL_USE_TRIPLE_BUFFER
#  HAGL_HAL_USE_DMA
  HAGL_HAL_DEBUG
#  HAGL_HAL_PIXEL_SIZE=2
)

include("generic-ili9341-320x240.cmake")

string(APPEND CMAKE_EXE_LINKER_FLAGS "-Wl,--print-memory-usage")

pico_add_extra_outputs(firmware)