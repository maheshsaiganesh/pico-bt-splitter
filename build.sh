#!/bin/sh
# Builds build/car_audio.uf2 (with USB serial for development).
#   ./build.sh flash      build + load onto the Pico (no BOOTSEL press needed)
#   ./build.sh car        car build: USB off (no buzz), output in build-car/
#   ./build.sh car flash  car build + load. Once a car build is running, hold
#                         BOOTSEL while plugging in to reflash.
# TARGET=boom_test ./build.sh flash   flash another program (default car_audio)
set -e
cd "$(dirname "$0")"
export PICO_SDK_PATH="$HOME/.pico-sdk/sdk-2.3.1"
export PICO_TOOLCHAIN_PATH="$HOME/.pico-sdk/arm-gnu-toolchain-14.2.rel1-darwin-arm64-arm-none-eabi"
BUILD=build CAR=OFF
if [ "$1" = "car" ]; then BUILD=build-car CAR=ON; shift; fi
cmake -S . -B "$BUILD" -G Ninja -DCMAKE_BUILD_TYPE=Release -DCAR=$CAR \
      -Dpicotool_DIR="$(brew --prefix picotool)/lib/cmake/picotool" >/dev/null
cmake --build "$BUILD"
if [ "$1" = "flash" ]; then
    # -f forces a running Pico (with USB stdio) into BOOTSEL first.
    picotool load -x -f "$BUILD/${TARGET:-car_audio}.uf2"
fi
