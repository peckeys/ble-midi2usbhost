# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.2.0)
set(toolchainVersion 14_2_Rel1)
set(picotoolVersion 2.2.0)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
cmake_minimum_required(VERSION 3.13)
set(target_name ble-midi2usbhost)
set(PICO_BOARD pico_w)
include (pico_sdk_import.cmake)
project(ble-midi2usbhost)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
pico_sdk_init()

add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/lib/ring_buffer_lib)
add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/lib/pico-w-ble-midi-lib)
add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/lib/usb_midi_host)

add_executable(ble-midi2usbhost
    ble-midi2usbhost.c
    )
target_link_libraries(ble-midi2usbhost
    pico_stdlib
    ble_midi_client_lib
    pico_btstack_cyw43
    pico_cyw43_arch_none
    ring_buffer_lib
    tinyusb_host
    tinyusb_board
    usb_midi_host_app_driver
    )
target_include_directories(ble-midi2usbhost PRIVATE
    ${CMAKE_CURRENT_LIST_DIR} # For btstack config, etc.
    )
pico_enable_stdio_uart(ble-midi2usbhost 1)
target_link_options(ble-midi2usbhost PRIVATE -Xlinker --print-memory-usage)
target_compile_options(ble-midi2usbhost PRIVATE -Wall -Wextra)
pico_btstack_make_gatt_header(ble-midi2usbhost PRIVATE "${CMAKE_CURRENT_LIST_DIR}/ble-midi2usbhost.gatt" "${CMAKE_CURRENT_LIST_DIR}/lib/pico-w-ble-midi-lib")

pico_add_extra_outputs(ble-midi2usbhost)

