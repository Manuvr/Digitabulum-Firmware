vpath %.c Source

# Includes
INCLUDES  = -I../Core/cmsis -I../Core/stm32 -I../../../Drivers/CMSIS/Device/ST/STM32F7xx/Include
INCLUDES += -Isrc -I../src/drivers -I../../../Inc -I../../../Drivers/STM32F7xx_HAL_Driver/Inc
INCLUDES += -I../../../Drivers/CMSIS/Include -ISource/CMSIS_RTOS -ISource/include
INCLUDES += -ISource/portable/GCC/ARM_CM7/r0p1/

# Sources
SRCS  = Source/timers.c Source/tasks.c Source/queue.c
SRCS += Source/list.c Source/event_groups.c Source/croutine.c
SRCS += Source/CMSIS_RTOS/cmsis_os.c
SRCS += Source/portable/GCC/ARM_CM7/r0p1/port.c
SRCS += Source/portable/MemMang/heap_4.c

OBJS = $(SRCS:.c=.o)
LIBNAME = libfreertos.a

all: $(LIBNAME)

%.o : Source/%.c
	$(CC) $(CFLAGS) $(INCLUDES) -nostdlib -c -o $@ $^

$(LIBNAME): $(OBJS)
	$(AR) -r $(OUTPUT_PATH)/$@ $(OBJS)

clean:
	rm -f $(OBJS) *.o
