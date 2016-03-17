# Build all libraries

all:
	$(MAKE) -C Drivers
	$(MAKE) -C Middlewares
	$(MAKE) -C telehash-c

clean:
	rm -f *.a
	$(MAKE) clean -C Drivers
	$(MAKE) clean -C Middlewares
	$(MAKE) clean -C telehash-c
