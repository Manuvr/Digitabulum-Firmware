#
# Digitabulum-r2 via ESP32
#

#COMPONENT_ADD_INCLUDEDIRS := include
#COMPONENT_PRIV_INCLUDEDIRS := include/freertos

#CFLAGS +=

COMPONENT_ADD_INCLUDEDIRS := $(PROJECT_PATH)/lib/ManuvrOS/ManuvrOS $(PROJECT_PATH)/lib

CFLAGS += -D__MANUVR_ESP32
CPPFLAGS += -D__MANUVR_ESP32

COMPONENT_SRCDIRS := CPLDDriver LSM9DS1 ManuLegend DigitabulumPMU
#COMPONENT_ADD_LDFLAGS := -L$(OUTPUT_PATH)/Digitabulum

COMPONENT_OBJS := CPLDDriver/CPLDDriver.o LSM9DS1/LSM9DS1.o LSM9DS1/RegPtrMap.o ManuLegend/SensorFrame.o ManuLegend/Integrator.o ManuLegend/ManuManager.o ManuLegend/ManuLegend.o DigitabulumPMU/DigitabulumPMU-r2.o
