# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Initialise pico_sdk from installed location
# (note this can come from environment, CMake cache etc)
set(PICO_SDK_PATH "/Users/shuichi/pico/pico-sdk")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(led_disp C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Add executable. Default name is the project name, version 0.1

add_executable(led_disp 
    main.cpp
    video_capture.cpp
    video_stream.cpp
    video_stream_buffer.cpp
    image_proc.cpp
    framebuffer.cpp
    adv7181.cpp
    pca9554.cpp
    rgb2serial.S
    ycbcr2rgb.S
)

pico_generate_pio_header(led_disp ${CMAKE_CURRENT_LIST_DIR}/led.pio)
pico_generate_pio_header(led_disp ${CMAKE_CURRENT_LIST_DIR}/bt656.pio)

pico_set_program_name(led_disp "led_disp")
pico_set_program_version(led_disp "0.1")

pico_enable_stdio_uart(led_disp 1)
pico_enable_stdio_usb(led_disp 0)

# Add the standard library to the build
target_link_libraries(led_disp
    pico_stdlib
    pico_multicore
    hardware_dma
    hardware_pio
    hardware_interp
    hardware_i2c
)

pico_add_extra_outputs(led_disp)

