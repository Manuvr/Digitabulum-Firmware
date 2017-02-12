################################################################################
# Makefile for Digitabulum
# Author: J. Ian Lindsay
# Date:   2017.02.11
#
# Currently supported targets:
#     make MANUVR_PLATFORM=LINUX
#     make MANUVR_PLATFORM=ESP32
#     make MANUVR_PLATFORM=STM32F7
################################################################################

################################################################################
## ESP32
ifeq ($(PLATFORM),ESP32)
ifndef IDF_PATH
	$(error If building for ESP32, you must supply the IDF_PATH variable.)
endif

export MANUVR_PLATFORM = ESP32

BUILD_ROOT           := $(shell pwd)
PROJECT_NAME         := digitabulum-r2
COMPONENT_SRCDIRS    := ManuvrOS/
BUILD_DIR_BASE       := $(OUTPUT_PATH)
EXTRA_COMPONENT_DIRS := $(BUILD_ROOT)/lib/
SRCDIRS              := $(BUILD_ROOT)/src/

# Pull in the esp-idf...
include $(IDF_PATH)/make/project.mk
endif   # ESP32


################################################################################
## Linux target
ifeq ($(PLATFORM),LINUX)
export MANUVR_PLATFORM = LINUX
export FIRMWARE_NAME   = "digitabulum-r2"
export BUILD_ROOT      = $(shell pwd)
export OUTPUT_PATH     = $(BUILD_ROOT)/build

include src/Targets/scripts/linux.mk
endif   # LINUX
