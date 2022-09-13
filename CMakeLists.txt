# Basic CMake setup
cmake_minimum_required(VERSION 3.19)
file(WRITE "${CMAKE_BINARY_DIR}/CACHEDIR.TAG" "Signature: 8a477f597d28d172789f06886806bc55")


# Include SDK and rust library
include(pico-sdk/pico_sdk_init.cmake)
add_subdirectory(rust)


# Define project
project(template C CXX ASM) # FIXME
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
add_compile_options(-Wall -Wextra -Werror)


# Init SDK
pico_sdk_init()


# Define binary
file(
    GLOB SRC_FILES
    ${PROJECT_SOURCE_DIR}/c/*.c)
add_executable(${PROJECT_NAME} ${SRC_FILES})

# Enable if you want to include PIO files
#file(
#    GLOB PIO_FILES
#    ${PROJECT_SOURCE_DIR}/c/*.pio
#    ${PROJECT_SOURCE_DIR}/c/**/*.pio)
#pico_generate_pio_header(${PROJECT_NAME} ${PIO_FILES})

add_compile_definitions(
    ${PROJECT_NAME}
    PICO_USE_STACK_GUARDS=1
    PICO_QUEUE_MAX_LEVEL=1)

target_link_libraries(
    ${PROJECT_NAME}
    pico_stdlib
    pico_multicore
    hardware_pio
    hardware_i2c
    hardware_interp
    hardware_pwm
    hardware_dma
    hardware_adc
    hardware_spi
    rust)


# Enable USB and build map/bin/hex file etc.
pico_enable_stdio_usb(${PROJECT_NAME} 1)
pico_enable_stdio_uart(${PROJECT_NAME} 1)
pico_enable_stdio_semihosting(${PROJECT_NAME} 0)
pico_add_extra_outputs(${PROJECT_NAME})
