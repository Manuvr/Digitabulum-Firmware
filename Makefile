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

OUTPUT_PATH        = build/
STLINK_LOADER_PATH = compiler/stlink

MCU                = cortex-m7
EXT_CLK_RATE       = 24000000
OPTIMIZATION       = -Os
C_STANDARD         = gnu99
CPP_STANDARD       = gnu++11


###########################################################################
# Environmental awareness...
###########################################################################
SHELL          = /bin/sh
WHO_I_AM       = $(shell whoami)
WHERE_I_AM     = $(shell pwd)
HOME_DIRECTORY = /home/$(WHO_I_AM)
TOOLCHAIN      = compiler/bin

CC_CROSS       = $(TOOLCHAIN)/arm-none-eabi-gcc
CPP_CROSS      = $(TOOLCHAIN)/arm-none-eabi-g++
LD_CROSS       = $(TOOLCHAIN)/arm-none-eabi-ld
AR_CROSS       = $(TOOLCHAIN)/arm-none-eabi-ar
AS_CROSS       = $(TOOLCHAIN)/arm-none-eabi-as
CP_CROSS       = $(TOOLCHAIN)/arm-none-eabi-objcopy
OD_CROSS       = $(TOOLCHAIN)/arm-none-eabi-objdump
SZ_CROSS       = $(TOOLCHAIN)/arm-none-eabi-size


###########################################################################
# Source files, includes, and linker directives...
###########################################################################
INCLUDES    = -iquote. -iquotesrc/ 
INCLUDES   += -Icompiler/arm-none-eabi/include/ 
INCLUDES   += -Ilib/Drivers/STM32F7xx_HAL_Driver/Inc/ -Ilib/Inc -Ilib/Drivers/CMSIS/Device/ST/STM32F7xx/Include
INCLUDES   += -Ilib/Drivers/CMSIS/Include
INCLUDES   += -Ilib/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS
INCLUDES   += -Ilib/Middlewares/Third_Party/FreeRTOS/Source/include
INCLUDES   += -Ilib/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM7/r0p1
INCLUDES   += -Ilib/Middlewares/Third_Party/FatFs/src
INCLUDES   += -Ilib/Middlewares/Third_Party/FatFs/src/drivers

# Library paths
LIBPATHS  = -L. -Llib/

# Libraries to link
LIBS = -lm -lstdperiph -lfatfs -lfreertos -lc

# Wrap the include paths into the flags...
CFLAGS = $(INCLUDES)
#CFLAGS += -g -ggdb
CFLAGS += $(OPTIMIZATION) -Wall -Tdigitabulum.ld

CFLAGS += -DHSE_VALUE=$(EXT_CLK_RATE)

# This will cause us to ignore the external OSC!!
CFLAGS += -DRUN_WITH_HSI

CFLAGS += -DSTM32F746xx -DREENTRANT_SYSCALLS_PROVIDED -DARM_MATH_CM7 -DUSE_STDPERIPH_DRIVER
CFLAGS += -DUSE_USB_OTG_FS
CFLAGS += -mlittle-endian -mthumb -mthumb-interwork -mcpu=cortex-m7
CFLAGS += -fsingle-precision-constant -Wdouble-promotion
CFLAGS += -mfpu=fpv5-sp-d16 -mfloat-abi=hard
CFLAGS += -ffreestanding


CPP_FLAGS = -std=$(CPP_STANDARD) -lstdc++
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


CFLAGS += $(CPP_FLAGS)


###########################################################################
# Source file definitions...
###########################################################################
SRCS    = src/main.c src/sdmmc src/spi.c src/syscalls.c src/tim.c src/usart.c src/usb_otg
SRCS   += src/bsp_driver_sd.c src/fatfs.c src/freertos.c src/gpio.c src/i2c.c src/rng.c 
SRCS   += src/stm32f7xx_hal_msp.c src/stm32f7xx_it.c
SRCS   += lib/Drivers/CMSIS/Device/ST/STM32F7xx/Source/Templates/gcc/startup_stm32f746xx.s
SRCS   += lib/Drivers/CMSIS/Device/ST/STM32F7xx/Source/Templates/system_stm32f7xx.c

#CPP_SRCS  = src/*.cpp

#SRCS   += $(CPP_SRCS)


###################################################

vpath %.cpp src
vpath %.c src
vpath %.a lib


###########################################################################
# Rules for building the firmware follow...
###########################################################################
OBJS = $(SRCS:.c=.o)
.PHONY: lib $(OUTPUT_PATH)/$(FIRMWARE_NAME).elf


all: lib $(OUTPUT_PATH)/$(FIRMWARE_NAME).elf
	$(SZ_CROSS) $(OUTPUT_PATH)/$(FIRMWARE_NAME).elf


%.o : %.c
	$(CPP_CROSS) $(CFLAGS) -c -o $@ $^


lib:
	$(MAKE) -C lib


$(OUTPUT_PATH)/$(FIRMWARE_NAME).elf: $(OBJS)
	$(shell mkdir $(OUTPUT_PATH))
	$(CPP_CROSS) $(CFLAGS) $^ -o $@ $(OBJS) $(LIBPATHS) $(LIBS)
	$(CP_CROSS) -O ihex $(OUTPUT_PATH)/$(FIRMWARE_NAME).elf $(OUTPUT_PATH)/$(FIRMWARE_NAME).hex
	$(CP_CROSS) -O binary $(OUTPUT_PATH)/$(FIRMWARE_NAME).elf $(OUTPUT_PATH)/$(FIRMWARE_NAME).bin


program: $(OUTPUT_PATH)/$(FIRMWARE_NAME).elf
#	$(TOOLCHAIN)/arm-none-eabi-gdb $(OUTPUT_PATH)/$(FIRMWARE_NAME).elf --eval-command="tar extended-remote :4242" --eval-command="load"
	dfu-util -d 0483:df11 -a 0  -s 0x8000000 -D $(OUTPUT_PATH)/$(FIRMWARE_NAME).bin --reset


fullclean: clean
	rm -rf doc/doxygen/*
	$(MAKE) clean -C lib

clean:
	rm -f *.o *.su *~
	rm -rf $(OUTPUT_PATH)

doc:
	mkdir -p doc/doxygen/
	doxygen Doxyfile
