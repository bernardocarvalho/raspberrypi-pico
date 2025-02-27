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
set(sdkVersion 2.1.1)
set(toolchainVersion 14_2_Rel1)
set(picotoolVersion 2.1.1)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD pico2 CACHE STRING "Board type")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(external_adc C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Add executable. Default name is the project name, version 0.1

if (TARGET tinyusb_device)
    add_executable(external_adc
            external_adc.c
            )

    # pull in common dependencies
    target_link_libraries(external_adc pico_stdlib 
        hardware_dma hardware_pio)

    # enable usb output, disable uart output
    pico_enable_stdio_usb(external_adc 1)
    pico_enable_stdio_uart(external_adc 0)

    # create map/bin/hex/uf2 file etc.
    pico_add_extra_outputs(external_adc)

    # add url via pico_set_program_url
elseif(PICO_ON_DEVICE)
    message("Skipping external_adc because TinyUSB submodule is not initialized in the SDK")
endif()

pico_generate_pio_header(external_adc ${CMAKE_CURRENT_LIST_DIR}/adc_12bit_input.pio)

