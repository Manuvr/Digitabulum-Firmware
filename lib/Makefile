# Build all libraries
C_STANDARD   = gnu99
CPP_STANDARD = gnu++11

LOCAL_CXX_FLAGS = -nostdlib $(CPP_FLAGS)

SOURCES_CPP =

OBJS  = $(SOURCES_C:.c=.o) $(SOURCES_CPP:.cpp=.o)

LIBNAME = libextras.a


all: $(LIBNAME)
	$(MAKE) -C Drivers
	$(MAKE) -C Middlewares
	$(MAKE) -C ManuvrOS

%.o : %.c
	$(CC) $(INCLUDES) $(LOCAL_C_FLAGS) -std=$(C_STANDARD) -c $< -o $@

%.o : %.cpp
	$(CXX) $(INCLUDES) $(LOCAL_CXX_FLAGS) -std=$(CPP_STANDARD) -c $< -o $@

$(LIBNAME): $(OBJS)
	$(AR) -rvs $(OUTPUT_PATH)/$@ $(OBJS)

clean:
	rm -f $(OBJS)
	rm -f $(OUTPUT_PATH)/$(LIBNAME)
	$(MAKE) clean -C Drivers
	$(MAKE) clean -C Middlewares
	$(MAKE) clean -C ManuvrOS
