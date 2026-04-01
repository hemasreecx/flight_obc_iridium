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
set(sdkVersion 2.2.0)
set(toolchainVersion 14_2_Rel1)
set(picotoolVersion 2.2.0-a4)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD adafruit_feather_rp2040 CACHE STRING "Board type")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(obc_flight C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Add executable. Default name is the project name, version 0.1

add_executable(obc_flight  
                src/main.cpp
                src/drivers/kx134.cpp
                src/drivers/qmc5883l.cpp
                src/logging/kx_data_logger.cpp
                src/logging/qmc_data_logger.cpp
                src/modules/imu_acquisition.cpp
                src/modules/imu_conversion.cpp
                src/modules/mag_acquisition.cpp
                src/modules/mag_conversion.cpp
                src/rockblock_9603/iridium_driver.cpp
                src/rockblock_9603/rockblock_manager.cpp)

pico_set_program_name(obc_flight "obc_flight")
pico_set_program_version(obc_flight "0.1")

# Modify the below lines to enable/disable output over UART/USB
pico_enable_stdio_uart(obc_flight 0)
pico_enable_stdio_usb(obc_flight 1)

# Add the standard library to the build
target_link_libraries(obc_flight
        pico_stdlib
        hardware_i2c
        hardware_uart      # ← ADD THIS
        hardware_gpio      # ← ADD THIS
        hardware_timer
        hardware_irq
        hardware_flash     # ← ADD THIS — needed for CalibStore
        hardware_sync      # ← ADD THIS — needed for CalibStore
)

# Add the standard include files to the build
target_include_directories(obc_flight PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}/src
        ${CMAKE_CURRENT_LIST_DIR}/src/drivers
        ${CMAKE_CURRENT_LIST_DIR}/src/logging
        ${CMAKE_CURRENT_LIST_DIR}/src/app
        ${CMAKE_CURRENT_LIST_DIR}/src/modules
        ${CMAKE_CURRENT_LIST_DIR}/src/rockblock_9603
        )

pico_add_extra_outputs(obc_flight)

