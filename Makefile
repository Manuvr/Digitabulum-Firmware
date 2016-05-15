###########################################################################
# Makefile for Digitabulum.
# Author: J. Ian Lindsay
# Date:   2014.08.13
#
#
#
# Variables for the firmware compilation...
###########################################################################
FIRMWARE_NAME      = digitabulum

MCU                = cortex-m7
EXT_CLK_RATE       = 24000000
OPTIMIZATION       = -O0 -g
C_STANDARD         = gnu99
CPP_STANDARD       = gnu++11


###########################################################################
# Environmental awareness...
###########################################################################
WHERE_I_AM         = $(shell pwd)
TOOLCHAIN          = $(WHERE_I_AM)/compiler/bin
STLINK_LOADER_PATH = $(WHERE_I_AM)/compiler/stlink

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
export MAKE    = make


###########################################################################
# Source files, includes, and linker directives...
###########################################################################
INCLUDES    = -iquote. -iquotesrc/
INCLUDES   += -Icompiler/arm-none-eabi/include/
INCLUDES   += -I$(WHERE_I_AM)/lib/ManuvrOS
INCLUDES   += -I$(WHERE_I_AM)/lib/Drivers/STM32F7xx_HAL_Driver/Inc/
INCLUDES   += -I$(WHERE_I_AM)/lib/Inc
INCLUDES   += -I$(WHERE_I_AM)/lib/Drivers/CMSIS/Device/ST/STM32F7xx/Include
INCLUDES   += -I$(WHERE_I_AM)/lib/Drivers/CMSIS/Include
INCLUDES   += -I$(WHERE_I_AM)/lib/Drivers/USB_Device/Class/CDC/Inc
INCLUDES   += -I$(WHERE_I_AM)/lib/Drivers/USB_Device/Core/Inc
INCLUDES   += -Ilib/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS
INCLUDES   += -Ilib/Middlewares/Third_Party/FreeRTOS/Source/include
INCLUDES   += -Ilib/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM7/r0p1
INCLUDES   += -Ilib/Middlewares/Third_Party/FatFs/src
INCLUDES   += -Ilib/Middlewares/Third_Party/FatFs/src/drivers

# Describing the target arch....
MCUFLAGS  = -DHSE_VALUE=$(EXT_CLK_RATE)
MCUFLAGS += -DRUN_WITH_HSE
#MCUFLAGS += -DRUN_WITH_HSI
MCUFLAGS += -DSTM32F746xx -DARM_MATH_CM7
MCUFLAGS += -mlittle-endian -mthumb -mthumb-interwork -mcpu=$(MCU)
MCUFLAGS += -fsingle-precision-constant -Wdouble-promotion
MCUFLAGS += -mfpu=fpv5-sp-d16 -mfloat-abi=hard
MCUFLAGS += -ffreestanding

# Library paths
LIBPATHS  = -L. -Llib/

# Libraries to link
LIBS = -lm -lstdperiph -lfatfs -lfreertos -lc -lgcc -lstdc++

# Flags for the linker...
LDFLAGS  = -static $(MCUFLAGS)
LDFLAGS += $(LIBPATHS)
LDFLAGS += -Wl,--start-group $(LIBS) -Wl,--end-group
LDFLAGS += -Wl,--gc-sections -Wall -Tdigitabulum.ld
LDFLAGS += -Wl,-Map=$(FIRMWARE_NAME).map

# Wrap the include paths into the flags...
CFLAGS =  $(INCLUDES)
CFLAGS += -L$(OUTPUT_PATH)
CFLAGS += $(OPTIMIZATION) -Wall

# We include this specifically so that we can get a grip on configuration headers
#   downstream.
CFLAGS += -I$(WHERE_I_AM)/confs

CFLAGS += $(MCUFLAGS)

# This will cause us to ignore the external OSC!!
CFLAGS += -DREENTRANT_SYSCALLS_PROVIDED -DUSE_STDPERIPH_DRIVER
CFLAGS += -DENABLE_USB_VCP
#CFLAGS += -DHAL_CORTEX_MODULE_ENABLED

CFLAGS += -D__MANUVR_DEBUG

# Debug options.
#CFLAGS += -g -ggdb

CPP_FLAGS = -std=$(CPP_STANDARD) $(CFLAGS)
#CPP_FLAGS += -fno-use-linker-plugin
#CPP_FLAGS += -fno-rtti -fno-exceptions
#CPP_FLAGS += -fstack-usage

###########################################################################
# Are we on a 64-bit system? If so, we'll need to specify
#   that we want a 32-bit build...
# Thanks, estabroo...
# http://www.linuxquestions.org/questions/programming-9/how-can-make-makefile-detect-64-bit-os-679513/
###########################################################################
LBITS = $(shell getconf LONG_BIT)
ifeq ($(LBITS),64)
  TARGET_WIDTH = -m32
else
  TARGET_WIDTH =
endif

# Finally, export our flags for downstream Makefiles...
export CFLAGS
export CPP_FLAGS

export STM32F746xx

###########################################################################
# Source file definitions...
###########################################################################
SRCS    = src/syscalls.c src/tim.c src/gpio.c
SRCS   += src/bsp_driver_sd.c src/fatfs.c src/freertos.c
SRCS   += src/stm32f7xx_hal_msp.c src/stm32f7xx_it.c
SRCS   += src/system_stm32f7xx.c

CPP_SRCS  = src/main.cpp

# TODO: Need to understand why -l won't blend....
LIB_HARDCODES = $(OUTPUT_PATH)/*.a $(OUTPUT_PATH)/SensorPackage.o

###################################################

vpath %.cpp src
vpath %.c src
vpath %.a lib


###########################################################################
# Rules for building the firmware follow...
###########################################################################
OBJS = $(SRCS:.c=.o)

.PHONY: lib $(OUTPUT_PATH)/$(FIRMWARE_NAME).elf


all: $(OUTPUT_PATH)/$(FIRMWARE_NAME).elf
	$(SZ) $(OUTPUT_PATH)/$(FIRMWARE_NAME).elf


%.o : %.c
	$(CC) $(CFLAGS) -c -o $@ $^


lib: $(OBJS)
	mkdir -p $(OUTPUT_PATH)
	$(MAKE) -C lib


$(OUTPUT_PATH)/$(FIRMWARE_NAME).elf: lib
	$(shell mkdir $(OUTPUT_PATH))
	$(MAKE) -C src/Digitabulum/
	$(CXX) $(CPP_FLAGS) $(LDFLAGS) src/startup.s src/main.cpp $(OBJS) $(LIB_HARDCODES) -o $@
	$(CP) -O ihex $(OUTPUT_PATH)/$(FIRMWARE_NAME).elf $(OUTPUT_PATH)/$(FIRMWARE_NAME).hex
	$(CP) -O binary $(OUTPUT_PATH)/$(FIRMWARE_NAME).elf $(OUTPUT_PATH)/$(FIRMWARE_NAME).bin


program: $(OUTPUT_PATH)/$(FIRMWARE_NAME).elf
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
