BINPATH=../../../../compiler/bin/

CC=$(BINPATH)/arm-none-eabi-gcc
AR=$(BINPATH)/arm-none-eabi-ar

###################################################

vpath %.c src

CFLAGS  = -g -O2 -Wall
CFLAGS += -DSTM32F746xx -DREENTRANT_SYSCALLS_PROVIDED -DARM_MATH_CM7 -DUSE_STDPERIPH_DRIVER
CFLAGS += -mlittle-endian -mthumb -mthumb-interwork -mcpu=cortex-m7
CFLAGS += -fsingle-precision-constant -Wdouble-promotion
CFLAGS += -mfpu=fpv5-sp-d16 -mfloat-abi=hard
CFLAGS += -ffreestanding -nostdlib

# Includes
CFLAGS += -I../Core/cmsis -I../Core/stm32 -I../../../Drivers/CMSIS/Device/ST/STM32F7xx/Include
CFLAGS += -Isrc -I../src/drivers -I../../../Inc -I../../../Drivers/STM32F7xx_HAL_Driver/Inc
CFLAGS += -I../../../Drivers/CMSIS/Include -I../FreeRTOS/Source/CMSIS_RTOS -I../FreeRTOS/Source/include
CFLAGS += -I../FreeRTOS/Source/portable/GCC/ARM_CM7/r0p1/

# Sources
SRCS = ff.c ff_gen_drv.c diskio.c

OBJS = $(SRCS:.c=.o)
LIBNAME = libfatfs.a

all: $(LIBNAME)

%.o : src/%.c
	$(CC) $(CFLAGS) -c -o $@ $^

$(LIBNAME): $(OBJS)
	$(CC) -o option.o src/option/*.c $(CFLAGS)
	$(AR) -r ../../../$@ $(OBJS)

clean:
	rm -f $(OBJS) $(LIBNAME)
