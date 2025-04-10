# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Initialise pico_sdk from installed location
# (note this can come from environment, CMake cache etc)

# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.1.0)
set(toolchainVersion 13_3_Rel1)
set(picotoolVersion 2.1.0)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD pico CACHE STRING "Board type")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(pico_diffdrive_uros C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

link_directories(libmicroros)

# Add executable. Default name is the project name, version 0.1
add_executable(pico_diffdrive_uros
    pico_diffdrive_uros.c
    pico_uart_transport.c
)

pico_set_program_name(pico_diffdrive_uros "pico_diffdrive_uros")
pico_set_program_version(pico_diffdrive_uros "0.1")

SET(CMAKE_C_FLAGS  "${CMAKE_C_FLAGS} -ffunction-sections -fdata-sections")
SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -ffunction-sections -fdata-sections")

# Configure Pico
pico_enable_stdio_usb(pico_diffdrive_uros 1)
pico_enable_stdio_uart(pico_diffdrive_uros 0)
add_compile_definitions(PICO_UART_ENABLE_CRLF_SUPPORT=0)
add_compile_definitions(PICO_STDIO_ENABLE_CRLF_SUPPORT=0)
add_compile_definitions(PICO_STDIO_DEFAULT_CRLF=0)

# Add the standard library to the build
target_link_libraries(pico_diffdrive_uros pico_stdlib microros pico_time)

target_include_directories(pico_diffdrive_uros PUBLIC
    libmicroros/include
)

# Add the standard include files to the build
target_include_directories(pico_diffdrive_uros PRIVATE ${CMAKE_CURRENT_LIST_DIR})

# Generate PIO header
pico_generate_pio_header(pico_diffdrive_uros ${CMAKE_CURRENT_LIST_DIR}/quadrature_encoder.pio)

# Add any user requested libraries
target_link_libraries(pico_diffdrive_uros hardware_pio hardware_pwm hardware_spi hardware_gpio hardware_i2c)

pico_add_extra_outputs(pico_diffdrive_uros)

