#!/bin/bash
# Build AxxPD firmware using the STM32CubeIDE embedded toolchain.
# Requires STM32CubeIDE installed at the default Windows path.
set -e

export PATH="/c/ST/STM32CubeIDE_1.14.0/STM32CubeIDE/plugins/com.st.stm32cube.ide.mcu.externaltools.gnu-tools-for-stm32.13.3.rel1.win32_1.0.0.202411081344/tools/bin:/c/ST/STM32CubeIDE_1.14.0/STM32CubeIDE/plugins/com.st.stm32cube.ide.mcu.externaltools.make.win32_2.2.0.202409170845/tools/bin:$PATH"
cd "$(dirname "$0")/Debug"
make -j4 all 2>&1
