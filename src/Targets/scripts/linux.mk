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
export GCOV    = $(shell which gcov)


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
DRIVER_SRCS   = src/Targets/Linux/host-driver.cpp
FIRMWARE_SRCS = src/Targets/Linux/main-emu.cpp

CXX_SRCS   = src/Digitabulum/CPLDDriver/CPLDDriver.cpp
CXX_SRCS  += src/Digitabulum/LSM9DS1/LSM9DS1.cpp
CXX_SRCS  += src/Digitabulum/LSM9DS1/RegPtrMap.cpp
CXX_SRCS  += src/Digitabulum/ManuLegend/SensorFrame.cpp
CXX_SRCS  += src/Digitabulum/ManuLegend/Integrator.cpp
CXX_SRCS  += src/Digitabulum/ManuLegend/ManuManager.cpp
CXX_SRCS  += src/Digitabulum/ManuLegend/ManuLegend.cpp

###########################################################################
# Option conditionals
###########################################################################
MANUVR_OPTIONS += -DMANUVR_OVER_THE_WIRE
#MANUVR_OPTIONS += -DMANUVR_SUPPORT_OSC
MANUVR_OPTIONS += -DMANUVR_SUPPORT_TCPSOCKET
MANUVR_OPTIONS += -D__MANUVR_LINUX

# Enables ATECC provisioning-related features...
MANUVR_OPTIONS += -DATECC508_CAPABILITY_OTP_RW
MANUVR_OPTIONS += -DATECC508_CAPABILITY_CONFIG_UNLOCK
MANUVR_OPTIONS += -DMBEDTLS_CONFIG_FILE='<mbedTLS_conf.h>'

# Debugging options...
ifeq ($(SECURE),1)
  LIBS += -lmbedtls -lmbedx509 -lmbedcrypto
  export SECURE=1
endif

# Debugging options...
ifeq ($(DEBUG),1)
OPTIMIZATION    = -O0 -g
MANUVR_OPTIONS += -DMANUVR_DEBUG
#MANUVR_OPTIONS += -DMANUVR_PIPE_DEBUG
MANUVR_OPTIONS += -DMANUVR_IMU_DEBUG
MANUVR_OPTIONS += -DMANUVR_EVENT_PROFILER
CFLAGS         += -fprofile-arcs -ftest-coverage
#CFLAGS += -g -ggdb
#CXXFLAGS += -fno-use-linker-plugin
#CXXFLAGS += -fstack-usage
endif


###########################################################################
# exports, consolidation....
###########################################################################
# Groups of files...
OBJS          = $(C_SRCS:.c=.o) $(CXX_SRCS:.cpp=.o)
FIRMWARE_OBJS = $(FIRMWARE_SRCS:.cpp=.o)
DRIVER_OBJS   = $(DRIVER_SRCS:.cpp=.o)
COV_FILES     = $(OBJS:.o=.gcda) $(OBJS:.o=.gcno)
COV_FILES    += $(FIRMWARE_OBJS:.o=.gcda) $(FIRMWARE_OBJS:.o=.gcno)
COV_FILES    += $(DRIVER_OBJS:.o=.gcda) $(DRIVER_OBJS:.o=.gcno)

# Merge our choices and export them to the downstream Makefiles...
CFLAGS += $(MANUVR_OPTIONS) $(OPTIMIZATION) $(INCLUDES)
ANALYZER_FLAGS  = $(MANUVR_OPTIONS) $(INCLUDES)
ANALYZER_FLAGS += --std=c++11 --report-progress --force -j6

export MANUVR_PLATFORM = LINUX
export ANALYZER_FLAGS
export CFLAGS
export CXXFLAGS += $(CFLAGS)



###########################################################################
# Rules for building the firmware follow...
###########################################################################
vpath %.a $(OUTPUT_PATH)


.PHONY: all

all: firmware driver
	$(SZ) $(OUTPUT_PATH)/$(FIRMWARE_NAME)

%.o : %.c
	$(CC) $(CFLAGS) -std=$(C_STANDARD) -c -o $@ $^

%.o : %.cpp
	$(CXX) -std=$(CXX_STANDARD) $(CXXFLAGS) -c -o $@ $^

libs:
	mkdir -p $(OUTPUT_PATH)
	$(MAKE) -C lib/

firmware: $(OBJS) $(FIRMWARE_OBJS) libs
	$(CXX) $(FIRMWARE_OBJS) $(OBJS) -o $(OUTPUT_PATH)/$(FIRMWARE_NAME) $(CXXFLAGS) -std=$(CXX_STANDARD) $(LDFLAGS)

driver: $(OBJS) $(DRIVER_OBJS) libs
	$(CXX) $(DRIVER_OBJS) $(OBJS) -o $(OUTPUT_PATH)/demo-driver $(CXXFLAGS) -std=$(CXX_STANDARD) $(LDFLAGS)

coverage: $(OUTPUT_PATH)/$(FIRMWARE_NAME)
	#$(OUTPUT_PATH)/$(FIRMWARE_NAME) --run-tests
	$(GCOV) --demangled-names --preserve-paths --source-prefix $(BUILD_ROOT) $(CXX_SRCS) $(C_SRCS)
	lcov --capture --directory . --output-file coverage.info
	genhtml coverage.info --output-directory doc/coverage

clean:
	rm -rf $(OUTPUT_PATH)
	rm -f $(OBJS)


fullclean: clean
	rm -rf doc/doxygen/*
	$(MAKE) clean -C lib/
