# Build all libraries

.PHONY: all

all: $(LIBNAME)
	$(MAKE) -C Drivers
	$(MAKE) -C Middlewares
	$(MAKE) -C ManuvrOS

clean:
	rm -f $(OBJS)
	rm -f $(OUTPUT_PATH)/$(LIBNAME)
	$(MAKE) clean -C Drivers
	$(MAKE) clean -C Middlewares
	$(MAKE) clean -C ManuvrOS
