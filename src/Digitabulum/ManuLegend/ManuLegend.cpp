/*
File:   ManuLegend.cpp
Author: J. Ian Lindsay
Date:   2014.07.01

Copyright 2016 Manuvr, Inc

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.


*/

#include "ManuLegend.h"


/****************************************************************************************************
*   ___ _              ___      _ _              _      _
*  / __| |__ _ ______ | _ ) ___(_) |___ _ _ _ __| |__ _| |_ ___
* | (__| / _` (_-<_-< | _ \/ _ \ | / -_) '_| '_ \ / _` |  _/ -_)
*  \___|_\__,_/__/__/ |___/\___/_|_\___|_| | .__/_\__,_|\__\___|
*                                          |_|
* Constructors/destructors, class initialization functions and so-forth...
****************************************************************************************************/


/**
* Constructor
*/
ManuLegend::ManuLegend() {
  // Make the Legend NULL at first. Nothing requested.
  for (uint8_t idx = 0; idx < LEGEND_DATASET_IIU_COUNT; idx++) per_iiu_data[idx] = 0;
}

/**
* Destructor
*/
ManuLegend::~ManuLegend() {
  if (dataset_local) {
    free(dataset_local);
    dataset_local = nullptr;
    ds_size = 0;
  }
}



/**
* Call this fxn to finalize our choices about the Legend and allocate the memory.
*/
int8_t ManuLegend::finallize() {
  if (datasetSize() > 0) {
    if (dataset_local) {
      // We have already carved out space for a dataset. We will need to re-allocate.
      free(dataset_local);
      dataset_local = nullptr;
    }
    dataset_local = (uint8_t*) malloc(ds_size);
    if (dataset_local) {
      for (uint16_t i = 0; i < ds_size; i++) *(dataset_local + i) = 0;
      return 0;
    }
  }
  return -1;  // Failure.
}


/**
* Calling this will cause the class to copy the data the owner requested from the global data map.
*/
int8_t ManuLegend::copy_frame() {
  if ((nullptr == dataset_global) || (nullptr == dataset_local)) return -1;  // Bailout clause.
  StringBuilder output;
  uint16_t accumulated_offset = 0;
  if (sequence()) {
    // Sequence number is special, because it is specific to this copy of the map.
    (*((uint32_t*) dataset_local))++;
    accumulated_offset++;
  }
  if (deltaT()) {
    *((uint32_t*) dataset_local + accumulated_offset) = *((uint32_t*) dataset_global + LEGEND_DATASET_OFFSET_DELTA_T/4);
    accumulated_offset++;
  }
  if (positionGlobal()) {
    *((uint32_t*) dataset_local + accumulated_offset + 0) = *((uint32_t*) dataset_global + 0 + LEGEND_DATASET_OFFSET_G_POSITION/4);
    *((uint32_t*) dataset_local + accumulated_offset + 1) = *((uint32_t*) dataset_global + 1 + LEGEND_DATASET_OFFSET_G_POSITION/4);
    *((uint32_t*) dataset_local + accumulated_offset + 2) = *((uint32_t*) dataset_global + 2 + LEGEND_DATASET_OFFSET_G_POSITION/4);
    accumulated_offset += 3;
  }

  for (uint8_t idx = 0; idx < LEGEND_DATASET_IIU_COUNT; idx++) {
    if (orientation(idx)) {
      *((uint32_t*) dataset_local + accumulated_offset + 0) = *((uint32_t*) dataset_global + LEGEND_DATASET_GLOBAL_SIZE/4 + 0 + LEGEND_DATASET_OFFSET_QUAT/4 + (LEGEND_DATASET_PER_IMU_SIZE/4 * idx));
      *((uint32_t*) dataset_local + accumulated_offset + 1) = *((uint32_t*) dataset_global + LEGEND_DATASET_GLOBAL_SIZE/4 + 1 + LEGEND_DATASET_OFFSET_QUAT/4 + (LEGEND_DATASET_PER_IMU_SIZE/4 * idx));
      *((uint32_t*) dataset_local + accumulated_offset + 2) = *((uint32_t*) dataset_global + LEGEND_DATASET_GLOBAL_SIZE/4 + 2 + LEGEND_DATASET_OFFSET_QUAT/4 + (LEGEND_DATASET_PER_IMU_SIZE/4 * idx));
      *((uint32_t*) dataset_local + accumulated_offset + 3) = *((uint32_t*) dataset_global + LEGEND_DATASET_GLOBAL_SIZE/4 + 3 + LEGEND_DATASET_OFFSET_QUAT/4 + (LEGEND_DATASET_PER_IMU_SIZE/4 * idx));
      accumulated_offset += 4;
    }
    if (accNullGravity(idx)) {
      *((uint32_t*) dataset_local + accumulated_offset + 0) = *((uint32_t*) dataset_global + LEGEND_DATASET_GLOBAL_SIZE/4 + 0 + LEGEND_DATASET_OFFSET_NULL_GRAV/4 + (LEGEND_DATASET_PER_IMU_SIZE/4 * idx));
      *((uint32_t*) dataset_local + accumulated_offset + 1) = *((uint32_t*) dataset_global + LEGEND_DATASET_GLOBAL_SIZE/4 + 1 + LEGEND_DATASET_OFFSET_NULL_GRAV/4 + (LEGEND_DATASET_PER_IMU_SIZE/4 * idx));
      *((uint32_t*) dataset_local + accumulated_offset + 2) = *((uint32_t*) dataset_global + LEGEND_DATASET_GLOBAL_SIZE/4 + 2 + LEGEND_DATASET_OFFSET_NULL_GRAV/4 + (LEGEND_DATASET_PER_IMU_SIZE/4 * idx));
      accumulated_offset += 3;
    }
    if (accRaw(idx)) {
      *((uint32_t*) dataset_local + accumulated_offset + 0) = *((uint32_t*) dataset_global + LEGEND_DATASET_GLOBAL_SIZE/4 + 0 + LEGEND_DATASET_OFFSET_ACC/4 + (LEGEND_DATASET_PER_IMU_SIZE/4 * idx));
      *((uint32_t*) dataset_local + accumulated_offset + 1) = *((uint32_t*) dataset_global + LEGEND_DATASET_GLOBAL_SIZE/4 + 1 + LEGEND_DATASET_OFFSET_ACC/4 + (LEGEND_DATASET_PER_IMU_SIZE/4 * idx));
      *((uint32_t*) dataset_local + accumulated_offset + 2) = *((uint32_t*) dataset_global + LEGEND_DATASET_GLOBAL_SIZE/4 + 2 + LEGEND_DATASET_OFFSET_ACC/4 + (LEGEND_DATASET_PER_IMU_SIZE/4 * idx));
      accumulated_offset += 3;
    }
    if (gyro(idx)) {
      *((uint32_t*) dataset_local + accumulated_offset + 0) = *((uint32_t*) dataset_global + LEGEND_DATASET_GLOBAL_SIZE/4 + 0 + LEGEND_DATASET_OFFSET_GYR/4 + (LEGEND_DATASET_PER_IMU_SIZE/4 * idx));
      *((uint32_t*) dataset_local + accumulated_offset + 1) = *((uint32_t*) dataset_global + LEGEND_DATASET_GLOBAL_SIZE/4 + 1 + LEGEND_DATASET_OFFSET_GYR/4 + (LEGEND_DATASET_PER_IMU_SIZE/4 * idx));
      *((uint32_t*) dataset_local + accumulated_offset + 2) = *((uint32_t*) dataset_global + LEGEND_DATASET_GLOBAL_SIZE/4 + 2 + LEGEND_DATASET_OFFSET_GYR/4 + (LEGEND_DATASET_PER_IMU_SIZE/4 * idx));
      accumulated_offset += 3;
    }
    if (mag(idx)) {
      *((uint32_t*) dataset_local + accumulated_offset + 0) = *((uint32_t*) dataset_global + LEGEND_DATASET_GLOBAL_SIZE/4 + 0 + LEGEND_DATASET_OFFSET_MAG/4 + (LEGEND_DATASET_PER_IMU_SIZE/4 * idx));
      *((uint32_t*) dataset_local + accumulated_offset + 1) = *((uint32_t*) dataset_global + LEGEND_DATASET_GLOBAL_SIZE/4 + 1 + LEGEND_DATASET_OFFSET_MAG/4 + (LEGEND_DATASET_PER_IMU_SIZE/4 * idx));
      *((uint32_t*) dataset_local + accumulated_offset + 2) = *((uint32_t*) dataset_global + LEGEND_DATASET_GLOBAL_SIZE/4 + 2 + LEGEND_DATASET_OFFSET_MAG/4 + (LEGEND_DATASET_PER_IMU_SIZE/4 * idx));
      accumulated_offset += 3;
    }
    if (velocity(idx)) {
      *((uint32_t*) dataset_local + accumulated_offset + 0) = *((uint32_t*) dataset_global + LEGEND_DATASET_GLOBAL_SIZE/4 + 0 + LEGEND_DATASET_OFFSET_VEL/4 + (LEGEND_DATASET_PER_IMU_SIZE/4 * idx));
      *((uint32_t*) dataset_local + accumulated_offset + 1) = *((uint32_t*) dataset_global + LEGEND_DATASET_GLOBAL_SIZE/4 + 1 + LEGEND_DATASET_OFFSET_VEL/4 + (LEGEND_DATASET_PER_IMU_SIZE/4 * idx));
      *((uint32_t*) dataset_local + accumulated_offset + 2) = *((uint32_t*) dataset_global + LEGEND_DATASET_GLOBAL_SIZE/4 + 2 + LEGEND_DATASET_OFFSET_VEL/4 + (LEGEND_DATASET_PER_IMU_SIZE/4 * idx));
      accumulated_offset += 3;
    }
    if (position(idx)) {
      *((uint32_t*) dataset_local + accumulated_offset + 0) = *((uint32_t*) dataset_global + LEGEND_DATASET_GLOBAL_SIZE/4 + 0 + LEGEND_DATASET_OFFSET_POSITION/4 + (LEGEND_DATASET_PER_IMU_SIZE/4 * idx));
      *((uint32_t*) dataset_local + accumulated_offset + 1) = *((uint32_t*) dataset_global + LEGEND_DATASET_GLOBAL_SIZE/4 + 1 + LEGEND_DATASET_OFFSET_POSITION/4 + (LEGEND_DATASET_PER_IMU_SIZE/4 * idx));
      *((uint32_t*) dataset_local + accumulated_offset + 2) = *((uint32_t*) dataset_global + LEGEND_DATASET_GLOBAL_SIZE/4 + 2 + LEGEND_DATASET_OFFSET_POSITION/4 + (LEGEND_DATASET_PER_IMU_SIZE/4 * idx));
      accumulated_offset += 3;
    }
    if (temperature(idx)) {
      *((uint32_t*) dataset_local + accumulated_offset + 0) = *((uint32_t*) dataset_global + LEGEND_DATASET_GLOBAL_SIZE/4 + 0 + LEGEND_DATASET_OFFSET_TEMP/4 + (LEGEND_DATASET_PER_IMU_SIZE/4 * idx));
      accumulated_offset++;
    }
    if (samplesAcc(idx)) {
      *((uint32_t*) dataset_local + accumulated_offset + 0) = *((uint32_t*) dataset_global + LEGEND_DATASET_GLOBAL_SIZE/4 + 0 + LEGEND_DATASET_OFFSET_SC_ACC/4 + (LEGEND_DATASET_PER_IMU_SIZE/4 * idx));
      accumulated_offset++;
    }
    if (samplesGyro(idx)) {
      *((uint32_t*) dataset_local + accumulated_offset + 0) = *((uint32_t*) dataset_global + LEGEND_DATASET_GLOBAL_SIZE/4 + 0 + LEGEND_DATASET_OFFSET_SC_GYR/4 + (LEGEND_DATASET_PER_IMU_SIZE/4 * idx));
      accumulated_offset++;
    }
    if (samplesMag(idx)) {
      *((uint32_t*) dataset_local + accumulated_offset + 0) = *((uint32_t*) dataset_global + LEGEND_DATASET_GLOBAL_SIZE/4 + 0 + LEGEND_DATASET_OFFSET_SC_MAG/4 + (LEGEND_DATASET_PER_IMU_SIZE/4 * idx));
      accumulated_offset++;
    }
    if (samplesTemperature(idx)) {
      *((uint32_t*) dataset_local + accumulated_offset + 0) = *((uint32_t*) dataset_global + LEGEND_DATASET_GLOBAL_SIZE/4 + 0 + LEGEND_DATASET_OFFSET_SC_TMEP/4 + (LEGEND_DATASET_PER_IMU_SIZE/4 * idx));
      accumulated_offset++;
    }
  }

  Kernel::log(&output);
  return 0;
}


/*
* Calling this will cause the class to compile an aggregate size of the data it represents,
*   if it has not already done so.
*/
uint16_t ManuLegend::datasetSize() {
  if (ds_size > 0) return ds_size;
  uint16_t return_value = 0;
  for (uint8_t idx = 0; idx < LEGEND_DATASET_IIU_COUNT; idx++) if (accRaw(idx))             return_value += sizeof(Vector3<float>);
  for (uint8_t idx = 0; idx < LEGEND_DATASET_IIU_COUNT; idx++) if (gyro(idx))               return_value += sizeof(Vector3<float>);
  for (uint8_t idx = 0; idx < LEGEND_DATASET_IIU_COUNT; idx++) if (mag(idx))                return_value += sizeof(Vector3<float>);
  for (uint8_t idx = 0; idx < LEGEND_DATASET_IIU_COUNT; idx++) if (temperature(idx))        return_value += sizeof(float);
  for (uint8_t idx = 0; idx < LEGEND_DATASET_IIU_COUNT; idx++) if (orientation(idx))        return_value += sizeof(Quaternion);
  for (uint8_t idx = 0; idx < LEGEND_DATASET_IIU_COUNT; idx++) if (accNullGravity(idx))     return_value += sizeof(Vector3<float>);
  for (uint8_t idx = 0; idx < LEGEND_DATASET_IIU_COUNT; idx++) if (velocity(idx))           return_value += sizeof(Vector3<float>);
  for (uint8_t idx = 0; idx < LEGEND_DATASET_IIU_COUNT; idx++) if (position(idx))           return_value += sizeof(Vector3<float>);
  for (uint8_t idx = 0; idx < LEGEND_DATASET_IIU_COUNT; idx++) if (samplesAcc(idx))         return_value += sizeof(uint32_t);
  for (uint8_t idx = 0; idx < LEGEND_DATASET_IIU_COUNT; idx++) if (samplesGyro(idx))        return_value += sizeof(uint32_t);
  for (uint8_t idx = 0; idx < LEGEND_DATASET_IIU_COUNT; idx++) if (samplesMag(idx))         return_value += sizeof(uint32_t);
  for (uint8_t idx = 0; idx < LEGEND_DATASET_IIU_COUNT; idx++) if (samplesTemperature(idx)) return_value += sizeof(uint32_t);
  if (positionGlobal()) return_value += sizeof(Vector3<float>);
  if (sequence())       return_value += sizeof(uint32_t);
  if (deltaT())         return_value += sizeof(float);

  ds_size = return_value;
  return return_value;
}


/*
* When we re-configure the dataset, we typically need to broadcast the new Legend.
* Format is....
* Byte | Meaning
* -----|--------------------------
* 0    | IMU count
* 1    | Global data bitmask (LSB)
* 2    | Global data bitmask (MSB)
* 3    | IMU-0 data bitmask (LSB)
* 4    | IMU-0 data bitmask (MSB)
* 5    | IMU-1 data bitmask (LSB)
* 6    | IMU-2 data bitmask (MSB)
* ....and so on.
*/
void ManuLegend::formLegendString(StringBuilder *output) {
  output->concat((unsigned char) LEGEND_DATASET_IIU_COUNT);
  output->concat((uint8_t*) &_legend_flags, 2);
  output->concat((uint8_t*) per_iiu_data, (LEGEND_DATASET_IIU_COUNT * 2));
  output->string();   // Save a little memory.
}


void ManuLegend::printDebug(StringBuilder *output) {
  output->concatf("--------------------------------\n---Legend\n--------------------------------\n");
  output->concatf("-- dataset_global         0x%08x\n",    (unsigned long) dataset_global);
  output->concatf("-- dataset_local          0x%08x\n",    (unsigned long) dataset_local);
  output->concatf("-- dataset_size           %u\n",        (unsigned long) ds_size);

  output->concat("--- Enabled data:\n");
  if (sequence()) output->concatf("--- Sequence number     %u\n",        (unsigned long) *((uint32_t*) dataset_local));

  for (uint8_t idx = 0; idx < LEGEND_DATASET_IIU_COUNT; idx++) {
    output->concatf("\t IIU %02u \t", idx);
    if (accRaw(idx))             output->concat("accRaw  ");
    if (gyro(idx))               output->concat("gyro  ");
    if (mag(idx))                output->concat("mag  ");
    if (temperature(idx))        output->concat("temperature  ");
    if (orientation(idx))        output->concat("orientation  ");
    if (accNullGravity(idx))     output->concat("accNullGravity  ");
    if (velocity(idx))           output->concat("velocity  ");
    if (position(idx))           output->concat("position  ");
    if (samplesAcc(idx))         output->concat("samplesAcc  ");
    if (samplesGyro(idx))        output->concat("samplesGyro  ");
    if (samplesMag(idx))         output->concat("samplesMag  ");
    if (samplesTemperature(idx)) output->concat("samplesTemperature  ");
    output->concat("\n");
  }
}




void ManuLegend::printDataset(StringBuilder *output) {
  output->concat("--------------------------------\n--- Dataset\n--------------------------------\n");
  for (uint8_t i = 0; i < datasetSize()/4; i++) {
    output->concatf(" 0x%08x%s",  *((uint32_t*) (i + dataset_local)), (i%8 ? "" : "\n"));
  }
  output->concat("\n\n");
}
