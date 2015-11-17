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

# Describing the target arch....
MCUFLAGS  = -DHSE_VALUE=$(EXT_CLK_RATE) -DRUN_WITH_HSI
MCUFLAGS += -DSTM32F746xx -DARM_MATH_CM7 
MCUFLAGS += -mlittle-endian -mthumb -mthumb-interwork -mcpu=cortex-m7
MCUFLAGS += -fsingle-precision-constant -Wdouble-promotion
MCUFLAGS += -mfpu=fpv5-sp-d16 -mfloat-abi=hard
MCUFLAGS += -ffreestanding

# Library paths
LIBPATHS  = -L. -Llib/

# Libraries to link
LIBS = -lm -lfatfs -lstdperiph -lfreertos -lc -lgcc -lstdc++

# Flags for the linker...
LDFLAGS = -static $(MCUFLAGS)
LDFLAGS += $(LIBPATHS)
LDFLAGS += -Wl,--start-group $(LIBS) -Wl,--end-group
LDFLAGS += -Wl,--gc-sections -Wall -Tdigitabulum.ld --stats 
LDFLAGS += -Xlinker -Map -Xlinker $(FIRMWARE_NAME).map

# Wrap the include paths into the flags...
CFLAGS =  $(INCLUDES)
CFLAGS += $(OPTIMIZATION) -Wall

CFLAGS += $(MCUFLAGS)

# This will cause us to ignore the external OSC!!
CFLAGS += -DREENTRANT_SYSCALLS_PROVIDED -DUSE_STDPERIPH_DRIVER
CFLAGS += -DUSE_USB_OTG_FS

# Debug options.
CFLAGS += -g -ggdb
CFLAGS += -Wl,-Map=$(FIRMWARE_NAME).map


CPP_FLAGS = -std=$(CPP_STANDARD)
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
SRCS    = src/main.c src/sdmmc.c src/spi.c src/syscalls.c src/tim.c src/usart.c src/usb_otg.c
SRCS   += src/bsp_driver_sd.c src/fatfs.c src/freertos.c src/gpio.c src/i2c.c src/rng.c 
SRCS   += src/stm32f7xx_hal_msp.c src/stm32f7xx_it.c
SRCS   += src/system_stm32f7xx.c src/startup.s


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


$(OUTPUT_PATH)/$(FIRMWARE_NAME).elf: 
	$(shell mkdir $(OUTPUT_PATH))
	$(CPP_CROSS) -c $(CFLAGS) $(SRCS)
	$(CPP_CROSS) $(CFLAGS) $^ -o $@ *.o $(LDFLAGS)
	$(CP_CROSS) -O ihex $(OUTPUT_PATH)/$(FIRMWARE_NAME).elf $(OUTPUT_PATH)/$(FIRMWARE_NAME).hex
	$(CP_CROSS) -O binary $(OUTPUT_PATH)/$(FIRMWARE_NAME).elf $(OUTPUT_PATH)/$(FIRMWARE_NAME).bin


program: $(OUTPUT_PATH)/$(FIRMWARE_NAME).elf
#	$(TOOLCHAIN)/arm-none-eabi-gdb $(OUTPUT_PATH)/$(FIRMWARE_NAME).elf --eval-command="tar extended-remote :4242" --eval-command="load"
	dfu-util -d 0483:df11 -a 0  -s 0x08000000 -D $(OUTPUT_PATH)/$(FIRMWARE_NAME).bin --reset


fullclean: clean
	rm -rf doc/doxygen/*
	$(MAKE) clean -C lib

clean:
	rm -f *.o *.su *~ *.map
	rm -rf $(OUTPUT_PATH)

doc:
	mkdir -p doc/doxygen/
	doxygen Doxyfile
