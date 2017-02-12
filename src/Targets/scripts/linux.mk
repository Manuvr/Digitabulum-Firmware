###########################################################################
# Makefile for Digitabulum emulator.
# Author: J. Ian Lindsay
# Date:   2016.12.03
#
###########################################################################
OPTIMIZATION       = -O2
C_STANDARD         = gnu99
CXX_STANDARD       = gnu++11


###########################################################################
# Environmental awareness...
###########################################################################
# This is where we will store compiled libs and the final output.

export CC      = $(shell which gcc)
export CXX     = $(shell which g++)
export LD      = $(shell which ld)
export AR      = $(shell which ar)
export AS      = $(shell which as)
export CP      = $(shell which objcopy)
export OD      = $(shell which objdump)
export SZ      = $(shell which size)
export MAKE    = $(shell which make)


###########################################################################
# Includes, flags, and linker directives...
###########################################################################
CXXFLAGS     = -fno-rtti -fno-exceptions
CFLAGS       = -Wall
LIBS         = -lc -lm -lpthread -lmanuvr

# Enforce a 32-bit build.
CFLAGS      += -m32

INCLUDES    = -iquote. -iquotesrc/
INCLUDES   += -I$(BUILD_ROOT)/lib/ManuvrOS/ManuvrOS
INCLUDES   += -I$(BUILD_ROOT)/lib
INCLUDES   += -I$(BUILD_ROOT)/confs
INCLUDES   += -I$(BUILD_ROOT)/src/Digitabulum

# Flags for the linker...
LDFLAGS  = -static
LDFLAGS += $(LIBPATHS)
LDFLAGS += -Wl,--start-group $(LIBS) -Wl,--end-group
LDFLAGS += -Wl,--gc-sections -Wall
LDFLAGS += -Wl,-Map=$(FIRMWARE_NAME).map
LDFLAGS += -L. -L$(OUTPUT_PATH)

###########################################################################
# Source file definitions...
###########################################################################
CXX_SRCS   = src/Targets/Linux/main-emu.cpp
CXX_SRCS  += src/Digitabulum/CPLDDriver/CPLDDriver.cpp
CXX_SRCS  += src/Digitabulum/LSM9DS1/LSM9DS1.cpp
CXX_SRCS  += src/Digitabulum/LSM9DS1/RegPtrMap.cpp
CXX_SRCS  += src/Digitabulum/ManuLegend/SensorFrame.cpp
CXX_SRCS  += src/Digitabulum/ManuLegend/Integrator.cpp
CXX_SRCS  += src/Digitabulum/ManuLegend/ManuManager.cpp
CXX_SRCS  += src/Digitabulum/ManuLegend/ManuLegend.cpp


###########################################################################
# Option conditionals
###########################################################################
MANUVR_OPTIONS += -DMANUVR_STDIO
MANUVR_OPTIONS += -DMANUVR_CONSOLE_SUPPORT
MANUVR_OPTIONS += -DMANUVR_SUPPORT_I2C
MANUVR_OPTIONS += -DMANUVR_OVER_THE_WIRE
MANUVR_OPTIONS += -DMANUVR_SUPPORT_TCPSOCKET
MANUVR_OPTIONS += -DMANUVR_CBOR
MANUVR_OPTIONS += -DMANUVR_STORAGE
MANUVR_OPTIONS += -D__MANUVR_LINUX


# Debugging options...
ifeq ($(DEBUG),1)
MANUVR_OPTIONS += -D__MANUVR_DEBUG
#MANUVR_OPTIONS += -D__MANUVR_PIPE_DEBUG
MANUVR_OPTIONS += -D__IMU_DEBUG
MANUVR_OPTIONS += -D__MANUVR_EVENT_PROFILER
#CFLAGS += -g -ggdb
#CXXFLAGS += -fno-use-linker-plugin
#CXXFLAGS += -fstack-usage
endif


###########################################################################
# exports, consolidation....
###########################################################################
OBJS = $(C_SRCS:.c=.o) $(CXX_SRCS:.cpp=.o)

# Merge our choices and export them to the downstream Makefiles...
CFLAGS += $(MANUVR_OPTIONS) $(OPTIMIZATION) $(INCLUDES)

ANALYZER_FLAGS  = $(MANUVR_OPTIONS) $(INCLUDES)
ANALYZER_FLAGS +=  --std=c++11 --report-progress --force -j6

#export MANUVR_PLATFORM = LINUX
export CFLAGS
export CXXFLAGS += $(CFLAGS)
export ANALYZER_FLAGS


###########################################################################
# Rules for building the firmware follow...
###########################################################################
vpath %.a $(OUTPUT_PATH)


.PHONY: all

all: $(OUTPUT_PATH)/$(FIRMWARE_NAME)
	$(SZ) $(OUTPUT_PATH)/$(FIRMWARE_NAME)

%.o : %.c
	$(CC) $(CFLAGS) -std=$(C_STANDARD) -c -o $@ $^

%.o : %.cpp
	$(CXX) -std=$(CXX_STANDARD) $(CXXFLAGS) -c -o $@ $^

libs:
	mkdir -p $(OUTPUT_PATH)
	$(MAKE) -C lib

$(OUTPUT_PATH)/$(FIRMWARE_NAME): $(OBJS) libs
	$(CXX) $(OBJS) -o $@ $(CXXFLAGS) -std=$(CXX_STANDARD) $(LDFLAGS)

clean:
	rm -rf $(OUTPUT_PATH)
	rm -f $(OBJS)


fullclean: clean
	rm -rf doc/doxygen/*
	$(MAKE) clean -C lib/
