# Build all libraries

.PHONY: all

all: manuvr 

manuvr:
	$(MAKE) -C ManuvrOS/ManuvrOS

clean:
	rm -f $(OBJS)
	rm -f $(OUTPUT_PATH)/$(LIBNAME)
	$(MAKE) clean -C ManuvrOS/ManuvrOS
