# Build all libraries

.PHONY: all

all: manuvr stm32f7

manuvr:
	$(MAKE) -C ManuvrOS/ManuvrOS

stm32f7:
	$(MAKE) -C Drivers
	$(MAKE) -C Middlewares

clean:
	rm -f $(OBJS)
	rm -f $(OUTPUT_PATH)/$(LIBNAME)
	$(MAKE) clean -C Drivers
	$(MAKE) clean -C Middlewares
	$(MAKE) clean -C ManuvrOS/ManuvrOS
