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

add_compile_options(-fno-strict-aliasing)


#add_compile_definitions(BOARDTYPE_SINGLE)
add_compile_definitions(BOARDTYPE_SEPARATE_LED=1)
#add_compile_definitions(BOARDTYPE_SEPARATE_VIDEO=1)

# Add executable. Default name is the project name, version 0.1

add_executable(led_disp 
    main.cpp
    video_capture.cpp
    video_stream.cpp
    video_stream_buffer.cpp
    image_proc.cpp
    framebuffer.cpp
    textplane.cpp
    menu.cpp
    adv7181.cpp
    pca9554.cpp
    tpa2016.cpp
    nv_settings.cpp
    ili9341.cpp
    video_serializer.cpp
    parameter_communicator.cpp
    remote_video_capture.cpp
    rgb2serial.S
    ycbcr2rgb.S
    misc.S
)

pico_generate_pio_header(led_disp ${CMAKE_CURRENT_LIST_DIR}/led.pio)
pico_generate_pio_header(led_disp ${CMAKE_CURRENT_LIST_DIR}/bt656.pio)
pico_generate_pio_header(led_disp ${CMAKE_CURRENT_LIST_DIR}/simple_serial.pio)

pico_set_program_name(led_disp "led_disp")
pico_set_program_version(led_disp "0.1")

pico_enable_stdio_uart(led_disp 1)
pico_enable_stdio_usb(led_disp 0)

# Add the standard library to the build
target_link_libraries(led_disp
    pico_stdlib
    pico_multicore
    pico_i2c_slave
    hardware_dma
    hardware_pio
    hardware_interp
    hardware_i2c
    hardware_flash
    hardware_clocks
)

pico_add_extra_outputs(led_disp)

