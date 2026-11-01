# Import the installed Raspberry Pi Pico SDK.
if(NOT PICO_SDK_PATH AND DEFINED ENV{PICO_SDK_PATH})
    set(PICO_SDK_PATH "$ENV{PICO_SDK_PATH}")
endif()
if(NOT PICO_SDK_PATH)
    message(FATAL_ERROR "Set PICO_SDK_PATH to the root of the pico-sdk checkout")
endif()
include("${PICO_SDK_PATH}/external/pico_sdk_import.cmake")
