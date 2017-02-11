###########################################################################
# Makefile for Digitabulum on the ESP32.
# Author: J. Ian Lindsay
# Date:   2017.02.08
#
###########################################################################
PROJECT_NAME   := digitabulum-r2

BUILD_ROOT   := $(shell pwd)
BUILD_DIR_BASE := $(BUILD_ROOT)/build
#EXTRA_COMPONENT_DIRS := $(BUILD_ROOT)/lib/
SRCDIRS := $(BUILD_ROOT)/src/


############################################################################
## Environmental awareness...
############################################################################
ifndef IDF_PATH
$(error If building for ESP32, you must supply the IDF_PATH variable.)
endif


include $(IDF_PATH)/make/project.mk




#OPTIMIZATION       = -O2
#C_STANDARD         = gnu99
#CPP_STANDARD       = gnu++11
#
#
#
## NOTE: Assumption...
#export TOOLCHAIN_PATH  = $(IDF_PATH)/xtensa-esp32-elf
#
#export CC      = $(TOOLCHAIN_PATH)/bin/xtensa-esp32-elf-gcc
#export CXX     = $(TOOLCHAIN_PATH)/bin/xtensa-esp32-elf-g++
#export AR      = $(TOOLCHAIN_PATH)/bin/xtensa-esp32-elf-ar
#export SZ      = $(TOOLCHAIN_PATH)/bin/xtensa-esp32-elf-size
#export LD      = $(TOOLCHAIN_PATH)/bin/xtensa-esp32-elf-ld
#export AS      = $(TOOLCHAIN_PATH)/bin/xtensa-esp32-elf-as
#export CP      = $(TOOLCHAIN_PATH)/bin/xtensa-esp32-elf-objcopy
#export OD      = $(TOOLCHAIN_PATH)/bin/xtensa-esp32-elf-objdump
#export MAKE    = $(shell which make)
#
#
#
############################################################################
## Includes, flags, and linker directives...
############################################################################
#CPP_FLAGS    = -fno-rtti -fno-exceptions
#CFLAGS       = -Wall
#LIBS         = -lc -lm -lpthread -lfreertos -llwip -lmanuvr
#
#INCLUDES    = -iquote. -iquotesrc/
#INCLUDES   += -I$(IDF_PATH)/components/
#INCLUDES   += -I$(IDF_PATH)/components/freertos/include/freertos/
#INCLUDES   += -I$(BUILD_ROOT)/lib/ManuvrOS/ManuvrOS
#INCLUDES   += -I$(BUILD_ROOT)/lib
#INCLUDES   += -I$(BUILD_ROOT)/confs
#INCLUDES   += -I$(BUILD_ROOT)/src/Digitabulum
#
## Flags for the linker...
#LDFLAGS  = -static
#LDFLAGS += $(LIBPATHS)
#LDFLAGS += -Wl,--start-group $(LIBS) -Wl,--end-group
#LDFLAGS += -Wl,--gc-sections -Wall
#LDFLAGS += -Wl,-Map=$(FIRMWARE_NAME).map
#LDFLAGS += -L. -L$(OUTPUT_PATH)
#
############################################################################
## Source file definitions...
############################################################################
#SOURCES_CPP   = src/main-wroom32.cpp
#SOURCES_CPP  += src/Digitabulum/CPLDDriver/CPLDDriver.cpp
#SOURCES_CPP  += src/Digitabulum/LSM9DS1/LSM9DS1.cpp
#SOURCES_CPP  += src/Digitabulum/LSM9DS1/RegPtrMap.cpp
#SOURCES_CPP  += src/Digitabulum/ManuLegend/SensorFrame.cpp
#SOURCES_CPP  += src/Digitabulum/ManuLegend/Integrator.cpp
#SOURCES_CPP  += src/Digitabulum/ManuLegend/ManuManager.cpp
#SOURCES_CPP  += src/Digitabulum/ManuLegend/ManuLegend.cpp
#
#
############################################################################
## Option conditionals
############################################################################
#MANUVR_OPTIONS += -D__MANUVR_FREERTOS
#MANUVR_OPTIONS += -D__MANUVR_ESP32
#
#
## Debugging options...
##CFLAGS += -g -ggdb
##CPP_FLAGS += -fno-use-linker-plugin
##CPP_FLAGS += -fstack-usage
#endif
#
#
############################################################################
## exports, consolidation....
############################################################################
#OBJS = $(SOURCES_C:.c=.o)
#
## Merge our choices and export them to the downstream Makefiles...
#CFLAGS += $(MANUVR_OPTIONS) $(OPTIMIZATION) $(INCLUDES)
#
#export MANUVR_PLATFORM = ESP32
#export CFLAGS
#export CPP_FLAGS += $(CFLAGS)
#
#
############################################################################
## Rules for building the firmware follow...
############################################################################
#vpath %.a $(OUTPUT_PATH)
#
#
#.PHONY: all
#
#all: $(OUTPUT_PATH)/$(FIRMWARE_NAME)
#	$(SZ) $(OUTPUT_PATH)/$(FIRMWARE_NAME)
#
#%.o : %.c
#	$(CC) $(CFLAGS) -c -o $@ $^
#
#libs:
#	mkdir -p $(OUTPUT_PATH)
#	$(MAKE) manuvr -C lib
#
#$(OUTPUT_PATH)/$(FIRMWARE_NAME): $(OBJS) libs
#	$(CXX) $(OBJS) $(SOURCES_CPP) -o $@ $(CPP_FLAGS) -std=$(CPP_STANDARD) $(LDFLAGS)
#
#clean:
#	rm -f *.o *.su *~ *.map $(OBJS)
#
#fullclean: clean
#	rm -rf doc/doxygen/*
#	rm -rf $(OUTPUT_PATH)
#	$(MAKE) clean -C lib
#
