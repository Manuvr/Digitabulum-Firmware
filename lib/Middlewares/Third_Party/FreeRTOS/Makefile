BINPATH=../../../../compiler/bin/

CC=$(BINPATH)/arm-none-eabi-gcc
AR=$(BINPATH)/arm-none-eabi-ar

###################################################

vpath %.c src

CFLAGS  = -g -O2 -Wall
CFLAGS += -DSTM32F746xx -DREENTRANT_SYSCALLS_PROVIDED -DARM_MATH_CM4 -DUSE_STDPERIPH_DRIVER
CFLAGS += -mlittle-endian -mthumb -mthumb-interwork -mcpu=cortex-m7
CFLAGS += -fsingle-precision-constant -Wdouble-promotion
CFLAGS += -mfpu=fpv4-sp-d16 -mfloat-abi=hard
CFLAGS += -ffreestanding -nostdlib

# Includes
CFLAGS += -I../Core/cmsis -I../Core/stm32 -I../../../Drivers/CMSIS/Device/ST/STM32F7xx/Include
CFLAGS += -Isrc -I../src/drivers -I../../../Inc -I../../../Drivers/STM32F7xx_HAL_Driver/Inc
CFLAGS += -I../../../Drivers/CMSIS/Include -ISource/CMSIS_RTOS -ISource/include
CFLAGS += -ISource/portable/GCC/ARM_CM7/r0p1/

# Sources
SRCS = timers.c tasks.c queue.c list.c event_groups.c croutine.c

OBJS = $(SRCS:.c=.o)
LIBNAME = libfreertos.a

all: $(LIBNAME)

%.o : Source/%.c
	$(CC) $(CFLAGS) -c -o $@ $^

$(LIBNAME): $(OBJS)
	$(CC) $(CFLAGS) -c -o cmsis_os.o Source/CMSIS_RTOS/cmsis_os.c
	$(CC) $(CFLAGS) -c -o port.o Source/portable/GCC/ARM_CM7/r0p1/port.c
	$(AR) -r ../../../$@ $(OBJS)

clean:
	rm -f *.o
