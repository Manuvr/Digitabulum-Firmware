vpath %.c src

# Includes
INCLUDES  = -I../Core/cmsis -I../Core/stm32 -I../../../Drivers/CMSIS/Device/ST/STM32F7xx/Include
INCLUDES += -Isrc -I../src/drivers -I../../../Inc -I../../../Drivers/STM32F7xx_HAL_Driver/Inc
INCLUDES += -I../../../Drivers/CMSIS/Include -I../FreeRTOS/Source/CMSIS_RTOS -I../FreeRTOS/Source/include
INCLUDES += -I../FreeRTOS/Source/portable/GCC/ARM_CM7/r0p1/

# Sources
SRCS = ff.c ff_gen_drv.c diskio.c

OBJS = $(SRCS:.c=.o)
LIBNAME = libfatfs.a

all: $(LIBNAME)

%.o : src/%.c
	$(CC) $(CFLAGS) $(INCLUDES) -nostdlib -c -o $@ $^

$(LIBNAME): $(OBJS)
	$(AR) -r $(OUTPUT_PATH)/$@ $(OBJS)

clean:
	rm -f $(OBJS) $(LIBNAME)
