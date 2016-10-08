###########################################################################
# Makefile for Digitabulum.
# Author: J. Ian Lindsay
# Date:   2014.08.13
#
# Variables for the firmware compilation...
###########################################################################
FIRMWARE_NAME      = digitabulum
OPTIMIZATION       = -O2
C_STANDARD         = gnu99
CPP_STANDARD       = gnu++11

MCU                = cortex-m7
EXT_CLK_RATE       = 24000000

###########################################################################
# Environmental awareness...
###########################################################################
WHERE_I_AM         = $(shell pwd)
TOOLCHAIN          = $(WHERE_I_AM)/compiler/bin
STLINK_LOADER_PATH = $(WHERE_I_AM)/compiler/stlink
CPP_FLAGS          = -fno-rtti -fno-exceptions
CFLAGS             =

# This is where we will store compiled libs and the final output.
export OUTPUT_PATH  = $(WHERE_I_AM)/build

export CC      = $(TOOLCHAIN)/arm-none-eabi-gcc
export CXX     = $(TOOLCHAIN)/arm-none-eabi-g++
export LD      = $(TOOLCHAIN)/arm-none-eabi-ld
export AR      = $(TOOLCHAIN)/arm-none-eabi-ar
export AS      = $(TOOLCHAIN)/arm-none-eabi-as
export CP      = $(TOOLCHAIN)/arm-none-eabi-objcopy
export OD      = $(TOOLCHAIN)/arm-none-eabi-objdump
export SZ      = $(TOOLCHAIN)/arm-none-eabi-size
export MAKE    = $(shell which make)


###########################################################################
# Source files, includes, and linker directives...
###########################################################################
INCLUDES    = -iquote. -iquotesrc/
INCLUDES   += -Icompiler/arm-none-eabi/include/
INCLUDES   += -I$(WHERE_I_AM)/lib/ManuvrOS
INCLUDES   += -I$(WHERE_I_AM)/lib/Drivers/STM32F7xx_HAL_Driver/Inc
INCLUDES   += -I$(WHERE_I_AM)/lib/Inc
INCLUDES   += -I$(WHERE_I_AM)/lib
INCLUDES   += -I$(WHERE_I_AM)/lib/Drivers/CMSIS/Device/ST/STM32F7xx/Include
INCLUDES   += -I$(WHERE_I_AM)/lib/Drivers/CMSIS/Include
INCLUDES   += -I$(WHERE_I_AM)/lib/Drivers/USB_Device/Class/CDC/Inc
INCLUDES   += -I$(WHERE_I_AM)/lib/Drivers/USB_Device/Core/Inc
INCLUDES   += -I$(WHERE_I_AM)/src/Digitabulum
INCLUDES   += -Ilib/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS
INCLUDES   += -Ilib/Middlewares/Third_Party/FreeRTOS/Source/include
INCLUDES   += -Ilib/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM7/r0p1
INCLUDES   += -Ilib/Middlewares/Third_Party/FatFs/src
INCLUDES   += -Ilib/Middlewares/Third_Party/FatFs/src/drivers

# Describing the target arch....
#MCUFLAGS  = -DHSE_VALUE=$(EXT_CLK_RATE)
#MCUFLAGS += -DRUN_WITH_HSE
MCUFLAGS += -DRUN_WITH_HSI
MCUFLAGS += -DSTM32F746xx -DARM_MATH_CM7
MCUFLAGS += -mlittle-endian -mthumb -mthumb-interwork -mcpu=$(MCU)
MCUFLAGS += -fsingle-precision-constant -Wdouble-promotion
MCUFLAGS += -mfpu=fpv5-sp-d16 -mfloat-abi=hard
MCUFLAGS += -ffreestanding

# Library paths
LIBPATHS  = -L. -L$(OUTPUT_PATH)

# Libraries to link
LIBS = -lm -lfatfs -lfreertos -lc -lgcc -lstdc++

# Flags for the linker...
LDFLAGS  = -static $(MCUFLAGS)
LDFLAGS += $(LIBPATHS)
LDFLAGS += -Wl,--start-group $(LIBS) -Wl,--end-group
LDFLAGS += -Wl,--gc-sections -Wall -Tdigitabulum.ld
LDFLAGS += -Wl,-Map=$(FIRMWARE_NAME).map

# Wrap the include paths into the flags...
CFLAGS =  $(INCLUDES)
CFLAGS += $(OPTIMIZATION) -Wall

# We include this specifically so that we can get a grip on configuration headers
#   downstream.
CFLAGS += -I$(WHERE_I_AM)/confs

CFLAGS += $(MCUFLAGS)

# This will cause us to ignore the external OSC!!
CFLAGS += -DREENTRANT_SYSCALLS_PROVIDED -DUSE_STDPERIPH_DRIVER
CFLAGS += -DENABLE_USB_VCP
#CFLAGS += -DHAL_CORTEX_MODULE_ENABLED

#CFLAGS += -DMANUVR_SUPPORT_MQTT
MANUVR_OPTIONS += -DMANUVR_OVER_THE_WIRE
MANUVR_OPTIONS += -DMANUVR_CBOR
MANUVR_OPTIONS += -DMANUVR_CONSOLE_SUPPORT
MANUVR_OPTIONS += -D__MANUVR_EVENT_PROFILER

# Debugging options...
ifeq ($(DEBUG),1)
MANUVR_OPTIONS += -D__MANUVR_DEBUG
#MANUVR_OPTIONS += -D__MANUVR_PIPE_DEBUG
#CFLAGS += -g -ggdb
#CPP_FLAGS += -fno-use-linker-plugin
#CPP_FLAGS += -fstack-usage
endif

export STM32F746xx
export MANUVR_PLATFORM = STM32F7

###########################################################################
# Source file definitions...
###########################################################################
SRCS    = src/syscalls.c src/gpio.c
SRCS   += src/fatfs.c src/freertos.c
SRCS   += src/stm32f7xx_it.c
SRCS   += src/system_stm32f7xx.c

CPP_SRCS   = src/main.cpp
CPP_SRCS  += src/Digitabulum/CPLDDriver/CPLDDriver.cpp
CPP_SRCS  += src/Digitabulum/CPLDDriver/SPIBusOp.cpp
CPP_SRCS  += src/Digitabulum/LSM9DS1/IIU.cpp
CPP_SRCS  += src/Digitabulum/LSM9DS1/LSM9DS1.cpp
CPP_SRCS  += src/Digitabulum/LSM9DS1/LSM9DS1_AG.cpp
CPP_SRCS  += src/Digitabulum/LSM9DS1/LSM9DS1_M.cpp
CPP_SRCS  += src/Digitabulum/ManuLegend/LegendManager.cpp
CPP_SRCS  += src/Digitabulum/ManuLegend/ManuLegend.cpp
CPP_SRCS  += src/Digitabulum/SDCard/SDCard.cpp
CPP_SRCS  += src/Digitabulum/RovingNetworks/RNBase.cpp
CPP_SRCS  += src/Digitabulum/RovingNetworks/BTQueuedOperation.cpp
CPP_SRCS  += src/Digitabulum/RovingNetworks/RN4677/RN4677.cpp
CPP_SRCS  += src/Digitabulum/USB/STM32F7USB.cpp
CPP_SRCS  += src/Digitabulum/IREmitter/IREmitter.cpp
CPP_SRCS  += src/Digitabulum/HapticStrap/HapticStrap.cpp
CPP_SRCS  += src/Digitabulum/ExpansionPort/ExpansionPort.cpp
CPP_SRCS  += src/Digitabulum/DigitabulumPMU/DigitabulumPMU.cpp



###########################################################################
# exports, consolidation....
###########################################################################
OBJS = $(SRCS:.c=.o)

# Finally, export our flags for downstream Makefiles...
export CFLAGS += $(MANUVR_OPTIONS)
export CPP_FLAGS += -std=$(CPP_STANDARD) $(CFLAGS)

vpath %.cpp src
vpath %.c src
vpath %.a $(OUTPUT_PATH)

###########################################################################
# Rules for building the firmware follow...
###########################################################################

.PHONY: all

all: $(OUTPUT_PATH)/$(FIRMWARE_NAME).elf
	$(SZ) $(OUTPUT_PATH)/$(FIRMWARE_NAME).elf


%.o : %.c
	$(CC) $(CFLAGS) -c -o $@ $^


libs:
	mkdir -p $(OUTPUT_PATH)
	$(MAKE) -C lib


$(OUTPUT_PATH)/$(FIRMWARE_NAME).elf: $(OBJS) libs
	$(CXX) $(CPP_FLAGS) $(LDFLAGS) src/startup.s $(CPP_SRCS) $(OBJS) -lmanuvr -lstdperiph -o $@
	$(CP) -O ihex $(OUTPUT_PATH)/$(FIRMWARE_NAME).elf $(OUTPUT_PATH)/$(FIRMWARE_NAME).hex
	$(CP) -O binary $(OUTPUT_PATH)/$(FIRMWARE_NAME).elf $(OUTPUT_PATH)/$(FIRMWARE_NAME).bin


program:
#	$(TOOLCHAIN)/arm-none-eabi-gdb $(OUTPUT_PATH)/$(FIRMWARE_NAME).elf --eval-command="tar extended-remote :4242" --eval-command="load"
	dfu-util -d 0483:df11 -a 0  -s 0x08000000 -D $(OUTPUT_PATH)/$(FIRMWARE_NAME).bin --reset


fullclean: clean
	rm -rf doc/doxygen/*
	$(MAKE) clean -C lib

clean:
	rm -f *.o *.su *~ *.map $(OBJS)
	rm -rf $(OUTPUT_PATH)

doc:
	mkdir -p doc/doxygen/
	doxygen Doxyfile
