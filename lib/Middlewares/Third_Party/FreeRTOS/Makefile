vpath %.c src

# Includes
INCLUDES  = -I../Core/cmsis -I../Core/stm32 -I../../../Drivers/CMSIS/Device/ST/STM32F7xx/Include
INCLUDES += -Isrc -I../src/drivers -I../../../Inc -I../../../Drivers/STM32F7xx_HAL_Driver/Inc
INCLUDES += -I../../../Drivers/CMSIS/Include -ISource/CMSIS_RTOS -ISource/include
INCLUDES += -ISource/portable/GCC/ARM_CM7/r0p1/

# Sources
SRCS = timers.c tasks.c queue.c list.c event_groups.c croutine.c

OBJS = $(SRCS:.c=.o)
LIBNAME = libfreertos.a

all: $(LIBNAME)

%.o : Source/%.c
	$(CC) $(CFLAGS) $(INCLUDES) -nostdlib -c -o $@ $^

$(LIBNAME): $(OBJS)
	$(CC) $(CFLAGS) $(INCLUDES) -c -o cmsis_os.o Source/CMSIS_RTOS/cmsis_os.c
	$(CC) $(CFLAGS) $(INCLUDES) -c -o port.o Source/portable/GCC/ARM_CM7/r0p1/port.c
	$(CC) $(CFLAGS) $(INCLUDES) -c -o heap.o Source/portable/MemMang/heap_4.c
	$(AR) -r $(OUTPUT_PATH)/$@ $(OBJS)

clean:
	rm -f $(OBJS) *.o
