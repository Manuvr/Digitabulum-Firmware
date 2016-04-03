# Build all libraries

all:
	$(MAKE) -C Drivers
#	$(MAKE) -C mbed
	$(MAKE) -C Middlewares
	$(MAKE) -C telehash-c
	$(MAKE) -C ManuvrOS

clean:
	rm -f *.a
	$(MAKE) clean -C Drivers
#	$(MAKE) clean -C mbed
	$(MAKE) clean -C Middlewares
	$(MAKE) clean -C telehash-c
	$(MAKE) clean -C ManuvrOS
