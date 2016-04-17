CXX_STANDARD       = gnu++11

# Driver-specific flags.
LOCAL_FLAGS  =  -std=$(CXX_STANDARD) -nostdlib

# Sources
SRCS   = CPLDDriver/CPLDDriver.cpp
SRCS  += CPLDDriver/SPIBusOp.cpp
SRCS  += LSM9DS1/IIU.cpp
SRCS  += LSM9DS1/LSM9DS1.cpp
SRCS  += LSM9DS1/LSM9DS1_AG.cpp
SRCS  += LSM9DS1/LSM9DS1_M.cpp
SRCS  += ManuLegend/LegendManager.cpp
SRCS  += ManuLegend/ManuLegend.cpp
SRCS  += SDCard/SDCard.cpp


all:
	$(CXX) -r $(CPP_FLAGS) $(LOCAL_FLAGS) $(LIBS) $(SRCS) -o $(OUTPUT_PATH)/SensorPackage.o


clean:
	rm -f $(OUTPUT_PATH)/SensorPackage.o
