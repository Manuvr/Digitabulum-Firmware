###########################################################################
# Makefile for Digitabulum.
# Author: J. Ian Lindsay
# Date:   2014.08.13
#
###########################################################################
FIRMWARE_NAME      = digitabulum

MCU                = cortex-m7
EXT_CLK_RATE       = 24000000
OPTIMIZATION       = -O2
C_STANDARD         = gnu99
CPP_STANDARD       = gnu++11


###########################################################################
# Environmental awareness...
###########################################################################
# This is where we will store compiled libs and the final output.
export BUILD_ROOT   = $(shell pwd)
export OUTPUT_PATH  = $(BUILD_ROOT)/build

TOOLCHAIN          = $(BUILD_ROOT)/compiler/bin
STLINK_LOADER_PATH = $(BUILD_ROOT)/compiler/stlink

export CC      = $(TOOLCHAIN)/arm-none-eabi-gcc
export CXX     = $(TOOLCHAIN)/arm-none-eabi-g++
export LD      = $(TOOLCHAIN)/arm-none-eabi-ld
export AR      = $(TOOLCHAIN)/arm-none-eabi-ar
export AS      = $(TOOLCHAIN)/arm-none-eabi-as
export CP      = $(TOOLCHAIN)/arm-none-eabi-objcopy
export OD      = $(TOOLCHAIN)/arm-none-eabi-objdump
export SZ      = $(TOOLCHAIN)/arm-none-eabi-size
export GDB     = $(TOOLCHAIN)/arm-none-eabi-gdb
export MAKE    = $(shell which make)


###########################################################################
# Includes, flags, and linker directives...
###########################################################################
CPP_FLAGS    = -fno-rtti -fno-exceptions
CFLAGS       = -Wall #-nostdlib
LIBS         = -lc -lm -lstdperiph -lfatfs -lfreertos -lmanuvr
LD_FILE      = digitabulum.ld

INCLUDES    = -iquote. -iquotesrc/
INCLUDES   += -Icompiler/arm-none-eabi/include
INCLUDES   += -I$(BUILD_ROOT)/lib/ManuvrOS/ManuvrOS
INCLUDES   += -I$(BUILD_ROOT)/lib/Drivers/STM32F7xx_HAL_Driver/Inc
INCLUDES   += -I$(BUILD_ROOT)/lib/Inc
INCLUDES   += -I$(BUILD_ROOT)/lib
INCLUDES   += -I$(BUILD_ROOT)/confs
INCLUDES   += -I$(BUILD_ROOT)/lib/Drivers/CMSIS/Device/ST/STM32F7xx/Include
INCLUDES   += -I$(BUILD_ROOT)/lib/Drivers/CMSIS/Include
INCLUDES   += -I$(BUILD_ROOT)/lib/Drivers/USB_Device/Class/CDC/Inc
INCLUDES   += -I$(BUILD_ROOT)/lib/Drivers/USB_Device/Core/Inc
INCLUDES   += -I$(BUILD_ROOT)/src/Digitabulum
INCLUDES   += -I$(BUILD_ROOT)/lib/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS
INCLUDES   += -I$(BUILD_ROOT)/lib/Middlewares/Third_Party/FreeRTOS/Source/include
INCLUDES   += -I$(BUILD_ROOT)/lib/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM7/r0p1
INCLUDES   += -I$(BUILD_ROOT)/lib/Middlewares/Third_Party/FatFs/src
INCLUDES   += -I$(BUILD_ROOT)/lib/Middlewares/Third_Party/FatFs/src/drivers

# Describing the target arch....
#MCUFLAGS  = -DHSE_VALUE=$(EXT_CLK_RATE)
#MCUFLAGS += -DRUN_WITH_HSE
MCUFLAGS += -DRUN_WITH_HSI
MCUFLAGS += -DSTM32F746xx -DSTM32F7xx -DARM_MATH_CM7
MCUFLAGS += -mlittle-endian -mthumb -mthumb-interwork -mcpu=$(MCU)
MCUFLAGS += -fsingle-precision-constant -Wdouble-promotion
MCUFLAGS += -mfpu=fpv5-sp-d16 -mfloat-abi=hard
MCUFLAGS += -ffreestanding

# Flags for the linker...
LDFLAGS  = -static $(MCUFLAGS)
LDFLAGS += $(LIBPATHS)
LDFLAGS += -Wl,--start-group $(LIBS) -Wl,--end-group
LDFLAGS += -Wl,--gc-sections -Wall -T$(LD_FILE)
LDFLAGS += -Wl,-Map=$(FIRMWARE_NAME).map
LDFLAGS += -L. -L$(OUTPUT_PATH)

CFLAGS += $(MCUFLAGS)

# This will cause us to ignore the external OSC!!
CFLAGS += -DREENTRANT_SYSCALLS_PROVIDED -DUSE_STDPERIPH_DRIVER
CFLAGS += -DENABLE_USB_VCP
#CFLAGS += -DHAL_CORTEX_MODULE_ENABLED

###########################################################################
# Source file definitions...
###########################################################################
SOURCES_C     = src/syscalls.c
SOURCES_C    += src/fatfs.c
SOURCES_C    += src/stm32f7xx_it.c
SOURCES_C    += src/system_stm32f7xx.c

SOURCES_CPP   = src/Digitabulum/CPLDDriver/CPLDDriver.cpp
SOURCES_CPP  += src/Digitabulum/CPLDDriver/CPLDBusOp.cpp
SOURCES_CPP  += src/Digitabulum/LSM9DS1/IIU.cpp
SOURCES_CPP  += src/Digitabulum/LSM9DS1/LSM9DS1.cpp
SOURCES_CPP  += src/Digitabulum/LSM9DS1/LSM9DS1_AG.cpp
SOURCES_CPP  += src/Digitabulum/LSM9DS1/LSM9DS1_M.cpp
SOURCES_CPP  += src/Digitabulum/ManuLegend/LegendManager.cpp
SOURCES_CPP  += src/Digitabulum/ManuLegend/ManuLegend.cpp
SOURCES_CPP  += src/Digitabulum/SDCard/SDCard.cpp
SOURCES_CPP  += src/Digitabulum/RovingNetworks/RNBase.cpp
SOURCES_CPP  += src/Digitabulum/RovingNetworks/BTQueuedOperation.cpp
SOURCES_CPP  += src/Digitabulum/RovingNetworks/RN4677/RN4677.cpp
SOURCES_CPP  += src/Digitabulum/USB/STM32F7USB.cpp
SOURCES_CPP  += src/Digitabulum/IREmitter/IREmitter.cpp
SOURCES_CPP  += src/Digitabulum/HapticStrap/HapticStrap.cpp
SOURCES_CPP  += src/Digitabulum/ExpansionPort/ExpansionPort.cpp
SOURCES_CPP  += src/Digitabulum/DigitabulumPMU/DigitabulumPMU.cpp


###########################################################################
# Option conditionals
###########################################################################
#MANUVR_OPTIONS += -DMANUVR_SUPPORT_MQTT
MANUVR_OPTIONS += -DMANUVR_OVER_THE_WIRE
MANUVR_OPTIONS += -DMANUVR_CBOR
MANUVR_OPTIONS += -DMANUVR_CONSOLE_SUPPORT
MANUVR_OPTIONS += -DMANUVR_SUPPORT_I2C

# Options that build for certain threading models (if any).
ifeq ($(THREADS),1)
INCLUDES   += -Ilib/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS
INCLUDES   += -Ilib/Middlewares/Third_Party/FreeRTOS/Source/include
INCLUDES   += -Ilib/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM7/r0p1
SOURCES_C  += src/freertos.c
MANUVR_OPTIONS += -D__MANUVR_FREERTOS
export THREADS=1
endif

# Options for various security features.
ifeq ($(SECURE),1)
MANUVR_OPTIONS += -DWITH_BLIND_CRYPTO
export SECURE=1
endif

# Debugging options...
ifeq ($(DEBUG),1)
MANUVR_OPTIONS += -D__MANUVR_DEBUG
#MANUVR_OPTIONS += -D__MANUVR_PIPE_DEBUG
MANUVR_OPTIONS += -D__MANUVR_EVENT_PROFILER
#CFLAGS += -g -ggdb
#CPP_FLAGS += -fno-use-linker-plugin
#CPP_FLAGS += -fstack-usage
endif

ifeq ($(DISCO),1)
# In this case, we will be doing hardware debugging on the F7 discovery.
# So we should add source files and options to reflect this.
DIGITABULUM_BOARD = DISCOF7
SOURCES_CPP  += src/main-discof7.cpp
CFLAGS += -DSTM32F7xx
else
DIGITABULUM_BOARD = R1
SOURCES_CPP  += src/main.cpp
endif

###########################################################################
# exports, consolidation....
###########################################################################
OBJS = $(SOURCES_C:.c=.o)

# Merge our choices and export them to the downstream Makefiles...
CFLAGS += $(MANUVR_OPTIONS) $(OPTIMIZATION) $(INCLUDES) -D$(DIGITABULUM_BOARD)

export STM32F746xx
export MANUVR_PLATFORM = STM32F7
export CFLAGS
export CPP_FLAGS += $(CFLAGS)


###########################################################################
# Rules for building the firmware follow...
###########################################################################
vpath %.cpp src
vpath %.c src
vpath %.a $(OUTPUT_PATH)


.PHONY: all

all: $(OUTPUT_PATH)/$(FIRMWARE_NAME).elf
	@echo '======================================================'
	@echo 'Built binary for board:'
	@echo $(DIGITABULUM_BOARD)
	@echo '======================================================'
	$(SZ) $(OUTPUT_PATH)/$(FIRMWARE_NAME).elf

%.o : %.c
	$(CC) $(CFLAGS) -c -o $@ $^

libs:
	mkdir -p $(OUTPUT_PATH)
	$(MAKE) -C lib

$(OUTPUT_PATH)/$(FIRMWARE_NAME).elf: $(OBJS) libs
	$(CXX) src/startup.s $(OBJS) $(SOURCES_CPP) -o $@ $(CPP_FLAGS) -std=$(CPP_STANDARD) $(LDFLAGS)
	$(CP) -O ihex $(OUTPUT_PATH)/$(FIRMWARE_NAME).elf $(OUTPUT_PATH)/$(FIRMWARE_NAME).hex
	$(CP) -O binary $(OUTPUT_PATH)/$(FIRMWARE_NAME).elf $(OUTPUT_PATH)/$(FIRMWARE_NAME).bin

program:
#	$(TOOLCHAIN)/arm-none-eabi-gdb $(OUTPUT_PATH)/$(FIRMWARE_NAME).elf --eval-command="tar extended-remote :4242" --eval-command="load"
	dfu-util -d 0483:df11 -a 0  -s 0x08000000 -D $(OUTPUT_PATH)/$(FIRMWARE_NAME).bin --reset

clean:
	rm -f *.o *.su *~ *.map $(OBJS)

fullclean: clean
	rm -rf doc/doxygen/*
	rm -rf $(OUTPUT_PATH)
	$(MAKE) clean -C lib

doc:
	mkdir -p doc/doxygen/
	doxygen Doxyfile
