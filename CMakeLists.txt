cmake_minimum_required(VERSION 3.13)

set(PICO_BOARD pico_w CACHE STRING "Board type")
include(pico_sdk_import.cmake)

project(pico_car_audio C CXX ASM)
set(CMAKE_C_STANDARD 11)
pico_sdk_init()

add_executable(car_audio
    src/main.c
    src/pwm_audio.c
)
target_include_directories(car_audio PRIVATE src)
target_link_libraries(car_audio
    pico_stdlib
    pico_multicore
    hardware_pwm
    hardware_dma
    hardware_resets
    pico_cyw43_arch_none
)
# CAR=ON: no USB at all. USB data traffic (1 kHz polling) was audible as a
# buzz on the AUX output; on a power bank it was clean. A car build has no
# serial log and needs the BOOTSEL button to reflash.
option(CAR "Build for the car (USB off)" OFF)
if(CAR)
    pico_enable_stdio_usb(car_audio 0)
else()
    pico_enable_stdio_usb(car_audio 1)
endif()
pico_enable_stdio_uart(car_audio 0)
pico_add_extra_outputs(car_audio)

# Step 2: Pico W -> UE Boom over Bluetooth (A2DP source) test tones.
add_executable(boom_test src/boom_test.c)
target_include_directories(boom_test PRIVATE src config)
target_compile_definitions(boom_test PRIVATE CYW43_LWIP=0) # Bluetooth only, no Wi-Fi stack
target_link_libraries(boom_test
    pico_stdlib
    pico_cyw43_arch_threadsafe_background
    pico_btstack_classic
    pico_btstack_cyw43
    pico_btstack_sbc_encoder
)
pico_enable_stdio_usb(boom_test 1)
pico_enable_stdio_uart(boom_test 0)
pico_add_extra_outputs(boom_test)

# Steps 3+4: iPhone -> Pico (A2DP sink) -> both Booms (SBC forwarded) + AUX (decoded, PWM).
add_executable(hub src/hub.c src/pwm_audio.c)
target_include_directories(hub PRIVATE src config)
target_compile_definitions(hub PRIVATE CYW43_LWIP=0)
target_link_libraries(hub
    pico_stdlib
    pico_multicore
    hardware_pwm
    hardware_dma
    hardware_resets
    pico_cyw43_arch_threadsafe_background
    pico_btstack_classic
    pico_btstack_cyw43
    pico_btstack_sbc_decoder
)
if(CAR)
    pico_enable_stdio_usb(hub 0) # USB traffic was audible as a buzz on AUX
else()
    pico_enable_stdio_usb(hub 1)
endif()
pico_enable_stdio_uart(hub 0)
pico_add_extra_outputs(hub)
