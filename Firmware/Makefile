# AxxPD G4 firmware — standalone Makefile for command-line builds
# Compatible with STM32CubeIDE toolchain (arm-none-eabi-gcc 13.3)
#
# Builds two images (see bootloader/bl_layout.h for the flash map):
#   build/AxxPD_boot.elf/.bin  — bootloader @ 0x08000000 (24 KB)
#   build/AxxPD.elf/.bin       — application @ 0x08006000; the .bin has its
#                                size/CRC header patched for the bootloader
#
# SWD flashing (both images):
#   STM32_Programmer_CLI -c port=SWD reset=HWrst \
#     -w build/AxxPD_boot.bin 0x08000000 -w build/AxxPD.bin 0x08006000 -v -rst

# Toolchain
PREFIX = arm-none-eabi-
CC  = $(PREFIX)gcc
CXX = $(PREFIX)g++
AS  = $(PREFIX)gcc -x assembler-with-cpp
LD  = $(PREFIX)g++
SZ  = $(PREFIX)size
OBJCOPY = $(PREFIX)objcopy
# Header patcher (zlib CRC32). Windows installs name it `python`, Linux
# distros `python3` — override with `make PYTHON=python3` when needed.
PYTHON ?= python

# Target
TARGET = AxxPD
BL_TARGET = AxxPD_boot
BUILD_DIR = build

# CPU flags
CPU = -mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb

# C defines
C_DEFS = \
-DUSE_HAL_DRIVER \
-DSTM32G491xx \
-DUSE_FULL_LL_DRIVER

# App-only defines: relocate the vector table to the app base (the app runs
# behind the bootloader — see bootloader/bl_layout.h)
APP_DEFS = -DUSER_VECT_TAB_ADDRESS -DVECT_TAB_OFFSET=0x00006000U

# Include paths (shared between C and C++)
C_INCLUDES = \
-ICore/Inc \
-IDrivers/STM32G4xx_HAL_Driver/Inc \
-IDrivers/STM32G4xx_HAL_Driver/Inc/Legacy \
-IDrivers/CMSIS/Device/ST/STM32G4xx/Include \
-IDrivers/CMSIS/Include \
-IUSB_Device/App \
-IUSB_Device/Target \
-IMiddlewares/ST/STM32_USB_Device_Library/Core/Inc \
-IMiddlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc \
-IDrivers/LCD \
-IDrivers/UGUI \
-IDrivers/UGUI/Fonts

# C++ additional includes (pdsink/etl must NOT be in C path — etl/string.h shadows system string.h)
CXX_INCLUDES = \
-Iaxxpd_firmware \
-Ipdsink/src \
-Ipdsink/src/pd \
-I.

# C sources
C_SOURCES = \
Core/Src/main.c \
Core/Src/stm32g4xx_hal_msp.c \
Core/Src/stm32g4xx_it.c \
Core/Src/system_stm32g4xx.c \
Core/Src/syscalls.c \
Core/Src/sysmem.c \
Core/Src/app_header.c \
Core/Src/atomic_ops.c \
Core/Src/boot_selector.c \
Core/Src/buttons.c \
Core/Src/buzzer.c \
Core/Src/graph.c \
Core/Src/ina228.c \
Core/Src/menu_settings.c \
Core/Src/ntc.c \
Core/Src/settings.c \
Core/Src/uart.c \
Core/Src/ui.c \
Drivers/LCD/lcd.c \
Drivers/LCD/images.c \
Drivers/LCD/splash_logo.c \
Drivers/UGUI/ugui.c \
Drivers/UGUI/ugui_button.c \
Drivers/UGUI/ugui_checkbox.c \
Drivers/UGUI/ugui_image.c \
Drivers/UGUI/ugui_progress.c \
Drivers/UGUI/ugui_textbox.c \
Drivers/UGUI/Fonts/arial_17X18.c \
Drivers/UGUI/Fonts/arial_23X24.c \
Drivers/UGUI/Fonts/arial_35X26.c \
Drivers/UGUI/Fonts/arial_35X37.c \
Drivers/UGUI/Fonts/arial_49X37.c \
USB_Device/App/usb_device.c \
USB_Device/App/usbd_cdc_if.c \
USB_Device/App/usbd_desc.c \
USB_Device/Target/usbd_conf.c \
Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_core.c \
Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ctlreq.c \
Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ioreq.c \
Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Src/usbd_cdc.c \
$(wildcard Drivers/STM32G4xx_HAL_Driver/Src/*.c)

# C++ sources
CXX_SOURCES = \
axxpd_firmware/axxpd_main.cpp \
axxpd_firmware/ucpd_driver.cpp \
axxpd_firmware/app_dpm.cpp \
axxpd_firmware/cable_emu.cpp \
axxpd_firmware/cli.cpp \
pdsink/src/pd/port.cpp \
pdsink/src/pd/task.cpp \
pdsink/src/pd/prl.cpp \
pdsink/src/pd/pe.cpp \
pdsink/src/pd/tc.cpp \
pdsink/src/pd/dpm.cpp \
pdsink/src/pd/utils/dobj_utils.cpp

# Assembly source
ASM_SOURCES = Core/Startup/startup_stm32g491ccux.s

# Linker script
LDSCRIPT = STM32G491CCUX_FLASH.ld

# Compiler flags. -MMD -MP generates .d dependency files so header edits
# (fw_version.h, bl_layout.h, ...) rebuild their dependents — without this a
# version bump could ship with a stale app header.
DEPFLAGS = -MMD -MP
CFLAGS = $(CPU) $(C_DEFS) $(APP_DEFS) $(C_INCLUDES) -Iaxxpd_firmware -Ibootloader -Wall -fdata-sections -ffunction-sections -Os -g $(DEPFLAGS)
CXXFLAGS = $(CPU) $(C_DEFS) $(APP_DEFS) $(C_INCLUDES) $(CXX_INCLUDES) -Ibootloader -Wall -fdata-sections -ffunction-sections -fno-exceptions -fno-rtti -std=c++17 -Os -g $(DEPFLAGS)
ASFLAGS = $(CPU) -g3

# Linker flags
LDFLAGS = $(CPU) -T$(LDSCRIPT) -Wl,--gc-sections -Wl,-Map=$(BUILD_DIR)/$(TARGET).map \
          --specs=nano.specs --specs=nosys.specs -u _printf_float \
          -lc -lm -lstdc++ -lsupc++

# Generate object file paths
C_OBJS   = $(addprefix $(BUILD_DIR)/,$(C_SOURCES:.c=.o))
CXX_OBJS = $(addprefix $(BUILD_DIR)/,$(CXX_SOURCES:.cpp=.o))
ASM_OBJS = $(addprefix $(BUILD_DIR)/,$(ASM_SOURCES:.s=.o))
OBJS = $(C_OBJS) $(CXX_OBJS) $(ASM_OBJS)

# =============================================================================
# Bootloader (pure C, minimal HAL subset, own linker script @ 0x08000000).
# Reuses the app's USB CDC glue (usbd_desc/usbd_conf/usbd_cdc_if) and the
# ST USB device library. No VECT_TAB relocation — it boots at flash base.
# =============================================================================
BL_LDSCRIPT = bootloader/STM32G491CCUX_BOOT.ld

BL_C_SOURCES = \
bootloader/bl_main.c \
bootloader/bl_it.c \
Core/Src/system_stm32g4xx.c \
Core/Src/syscalls.c \
Core/Src/sysmem.c \
USB_Device/App/usb_device.c \
USB_Device/App/usbd_cdc_if.c \
USB_Device/App/usbd_desc.c \
USB_Device/Target/usbd_conf.c \
Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_core.c \
Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ctlreq.c \
Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ioreq.c \
Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Src/usbd_cdc.c \
Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal.c \
Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_cortex.c \
Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_rcc.c \
Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_rcc_ex.c \
Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_gpio.c \
Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_pwr.c \
Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_pwr_ex.c \
Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_flash.c \
Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_flash_ex.c \
Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_flash_ramfunc.c \
Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_pcd.c \
Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_pcd_ex.c \
Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_ll_usb.c

BL_ASM_SOURCES = Core/Startup/startup_stm32g491ccux.s

BL_CFLAGS = $(CPU) $(C_DEFS) $(C_INCLUDES) -Ibootloader -Wall \
            -fdata-sections -ffunction-sections -Os -g $(DEPFLAGS)
BL_LDFLAGS = $(CPU) -T$(BL_LDSCRIPT) -Wl,--gc-sections \
             -Wl,-Map=$(BUILD_DIR)/$(BL_TARGET).map \
             --specs=nano.specs --specs=nosys.specs -lc -lm

BL_C_OBJS   = $(addprefix $(BUILD_DIR)/boot/,$(BL_C_SOURCES:.c=.o))
BL_ASM_OBJS = $(addprefix $(BUILD_DIR)/boot/,$(BL_ASM_SOURCES:.s=.o))
BL_OBJS = $(BL_C_OBJS) $(BL_ASM_OBJS)

all: $(BUILD_DIR)/$(TARGET).bin $(BUILD_DIR)/$(BL_TARGET).bin
	$(SZ) $(BUILD_DIR)/$(TARGET).elf
	$(SZ) $(BUILD_DIR)/$(BL_TARGET).elf

$(BUILD_DIR)/$(TARGET).elf: $(OBJS)
	$(LD) $(OBJS) $(LDFLAGS) -o $@

# App binary with the bootloader header patched in (size + CRC32). This .bin
# is what the dashboard uploads and what SWD scripts flash to 0x08006000.
$(BUILD_DIR)/$(TARGET).bin: $(BUILD_DIR)/$(TARGET).elf tools/patch_app_header.py
	$(OBJCOPY) -O binary $< $@
	$(PYTHON) tools/patch_app_header.py $@

$(BUILD_DIR)/$(BL_TARGET).elf: $(BL_OBJS)
	$(CC) $(BL_OBJS) $(BL_LDFLAGS) -o $@

$(BUILD_DIR)/$(BL_TARGET).bin: $(BUILD_DIR)/$(BL_TARGET).elf
	$(OBJCOPY) -O binary $< $@

$(BUILD_DIR)/%.o: %.c
	@mkdir -p $(dir $@)
	$(CC) $(CFLAGS) -c $< -o $@

$(BUILD_DIR)/%.o: %.cpp
	@mkdir -p $(dir $@)
	$(CXX) $(CXXFLAGS) -c $< -o $@

$(BUILD_DIR)/%.o: %.s
	@mkdir -p $(dir $@)
	$(AS) $(ASFLAGS) -c $< -o $@

$(BUILD_DIR)/boot/%.o: %.c
	@mkdir -p $(dir $@)
	$(CC) $(BL_CFLAGS) -c $< -o $@

$(BUILD_DIR)/boot/%.o: %.s
	@mkdir -p $(dir $@)
	$(AS) $(ASFLAGS) -c $< -o $@

clean:
	rm -rf $(BUILD_DIR)

# Auto-generated header dependencies (see DEPFLAGS)
-include $(OBJS:.o=.d) $(BL_C_OBJS:.o=.d)

.PHONY: all clean
