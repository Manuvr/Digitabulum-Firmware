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

BUILD_ROOT           := $(shell pwd)

# Enables ATECC provisioning-related features...
#MANUVR_OPTIONS  = -DATECC508_CAPABILITY_OTP_RW
#MANUVR_OPTIONS += -DATECC508_CAPABILITY_CONFIG_UNLOCK
MANUVR_OPTIONS += -DCONFIG_MANUVR_BQ24155
MANUVR_OPTIONS += -DCONFIG_MANUVR_LTC294X
MANUVR_OPTIONS += -D__MANUVR_ESP32

# TODO: Doesn't work. Never did. Fix.
# Debugging options...
#ifeq ($(DEBUG),1)
#  MANUVR_OPTIONS += -DMANUVR_DEBUG
#  #MANUVR_OPTIONS += -DMANUVR_PIPE_DEBUG
#  MANUVR_OPTIONS += -DMANUVR_IMU_DEBUG
#  MANUVR_OPTIONS += -DMANUVR_EVENT_PROFILER
#endif
#export MANUVR_OPTIONS

export MANUVR_PLATFORM = ESP32
export OUTPUT_PATH     = $(BUILD_ROOT)/build

PROJECT_NAME         := digitabulum-r2
#COMPONENT_SRCDIRS    := ManuvrOS/
BUILD_DIR_BASE       := $(OUTPUT_PATH)
SRCDIRS              := src/Digitabulum src/Targets/ESP32 lib/ManuvrOS/ManuvrOS

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


################################################################################
## STM32F7 target
ifeq ($(PLATFORM),STM32F7)
export MANUVR_PLATFORM = STM32F7
export FIRMWARE_NAME   = "digitabulum-r2"
export BUILD_ROOT      = $(shell pwd)
export OUTPUT_PATH     = $(BUILD_ROOT)/build

include src/Targets/scripts/stm32.mk
endif   # STM32F7
