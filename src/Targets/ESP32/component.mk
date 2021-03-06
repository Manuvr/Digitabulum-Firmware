#
# Digitabulum-r2 via ESP32
#

#COMPONENT_ADD_INCLUDEDIRS := include
#COMPONENT_PRIV_INCLUDEDIRS := include/freertos

#CFLAGS +=

COMPONENT_ADD_INCLUDEDIRS := ../../../lib/ManuvrOS/ManuvrOS ../../../lib ../..

# Enables ATECC provisioning-related features...
MANUVR_OPTIONS += -D__MANUVR_ESP32

CFLAGS   += $(MANUVR_OPTIONS)
CXXFLAGS += $(MANUVR_OPTIONS)

#COMPONENT_ADD_LDFLAGS := -L$(OUTPUT_PATH)/ESP32

COMPONENT_OBJS := main-esp32.o
