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


/*******************************************************************************
*   ___ _              ___      _ _              _      _
*  / __| |__ _ ______ | _ ) ___(_) |___ _ _ _ __| |__ _| |_ ___
* | (__| / _` (_-<_-< | _ \/ _ \ | / -_) '_| '_ \ / _` |  _/ -_)
*  \___|_\__,_/__/__/ |___/\___/_|_\___|_| | .__/_\__,_|\__\___|
*                                          |_|
* Constructors/destructors, class initialization functions and so-forth...
*******************************************************************************/

/**
* Copy constructor
*/
ManuLegend::ManuLegend(const ManuLegend* src) {
  uint8_t* _ptr = (uint8_t*) src;
  // Make the Legend NULL at first. Nothing requested.
  for (uint8_t i = 0; i < sizeof(ManuLegend); i++) *(_ptr+i);
}


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
}


/*
* Calling this will cause the class to compile an aggregate size of the data it represents,
*   if it has not already done so.
*
* TODO: This is awful and slow and a style mess, despite the fact
*   that it does exactly what it is supposed to do.
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
  if (handPosition()) return_value += sizeof(Vector3<float>);
  if (sequence())     return_value += sizeof(uint32_t);
  if (deltaT())       return_value += sizeof(float);

  ds_size = return_value;
  return return_value;
}


/**
* When we re-configure the dataset, we typically need to broadcast the new Legend.
* Format is....
* Byte | Meaning
* -----|--------------------------
* 0    | IMU count
* 1    | Global data bitmask
* 2    | IMU-0 data bitmask (LSB)
* 3    | IMU-0 data bitmask (MSB)
* 4    | IMU-1 data bitmask (LSB)
* 5    | IMU-1 data bitmask (MSB)
* ....and so on.
*
* @return non-zero on error.
*/
int8_t ManuLegend::getLegendString(StringBuilder* output) {
  StringBuilder scratchpad;
  scratchpad.concat((unsigned char) LEGEND_DATASET_IIU_COUNT);
  scratchpad.concat((unsigned char) frame_data);
  scratchpad.concat((uint8_t*) per_iiu_data, (iiu_count << 1));
  scratchpad.string();   // Save a little memory.
  output->concatHandoff(&scratchpad);
  return 0;
}

/**
* When we re-configure the dataset, we typically need to broadcast the new Legend.
* Format is....
* TODO: Might-should longword align this data.
* Byte | Meaning
* -----|--------------------------
* 0    | IMU count
* 1    | Global data bitmask
* 2    | IMU-0 data bitmask (LSB)
* 3    | IMU-0 data bitmask (MSB)
* 4    | IMU-1 data bitmask (LSB)
* 5    | IMU-1 data bitmask (MSB)
* ....and so on.
*
* @return non-zero on error.
*/
int8_t ManuLegend::setLegendString(StringBuilder* input) {
  int8_t return_value = -1;
  if (input) {
    int len = input->length();
    if (2 <= len) {
      uint8_t* buf  = input->string();
      uint8_t count = *(buf + 0);
      if (LEGEND_DATASET_IIU_COUNT >= count) {
        if (len == (3 + (count << 1))) {
          frame_data = *(buf + 1);
          int i = 0;
          for (;i < count; i++) {
            per_iiu_data[i] = parseUint16Fromchars(buf+3+(i << 1));
          }
          for (;i < LEGEND_DATASET_IIU_COUNT; i++) {
            per_iiu_data[i] = 0;
          }
          ds_size = 0;
          iiu_count = count;
          return_value = 0;
          datasetSize();
        }
        else {
          // Declared and derived lengths don't match. Legent invalid.
          return_value = -4;
        }
      }
      else {
        // Too many IIUs. Legend invalid.
        return_value = -3;
      }
    }
    else {
      // Minimum Legend size not met. Legend invalid.
      return_value = -2;
    }
  }
  return return_value;
}


/**
* Does this Legend satisfy the given Legend?
*
* @param test is the ManuLegend that sets the benchmark.
* @return true if so.
*/
bool ManuLegend::satisfiedBy(ManuLegend* test) {
  if (frame_data != (frame_data & test->frame_data)) {
    return false;
  }
  for (int i = 0; i < LEGEND_DATASET_IIU_COUNT; i++) {
    if (per_iiu_data[i] = (per_iiu_data[i] & test->per_iiu_data[i])) {
      return false;
    }
  }
  return true;
}


/**
* Put ourselves into a state where we satisfy the given Legend.
* Note that this function never unsets data demand.
*
* @param test is the ManuLegend that sets the baseline.
* @return True if we changed our own Legend.
*/
bool ManuLegend::stackLegend(ManuLegend* test) {
  bool return_value = false;
  if (frame_data != (frame_data & test->frame_data)) {
    frame_data   = frame_data | test->frame_data;
    return_value = true;
  }
  for (int i = 0; i < LEGEND_DATASET_IIU_COUNT; i++) {
    if (per_iiu_data[i] != (per_iiu_data[i] & test->per_iiu_data[i])) {
      per_iiu_data[i] = per_iiu_data[i] | test->per_iiu_data[i];
      return_value = true;
    }
  }
  return return_value;
}

/**
* Reset the ManuLegend.
*
* @return True if we changed our own Legend.
*/
bool ManuLegend::zeroLegend() {
  bool return_value = false;
  if (frame_data) {
    frame_data   = 0;
    return_value = true;
  }
  for (int i = 0; i < LEGEND_DATASET_IIU_COUNT; i++) {
    if (per_iiu_data[i]) {
      per_iiu_data[i] = 0;
      return_value = true;
    }
  }
  return return_value;
}

/**
* Proceeding from the top of the data stack downward, fill in any implicit
*   data demands. This should only be invoked for a ManuLegend that is being
*   fed to the integrator, or some other such application. ManuLegendPipes that
*   call this method will construe it as opening a data filter.
*
* @return True if we changed our own Legend.
*/
bool ManuLegend::fillLegendGaps() {
  bool return_value = false;
  for (uint8_t i = 0; i < LEGEND_DATASET_IIU_COUNT; i++) {
    if (handPosition() || position(i)) {
      // Asking for IMU position is asking for everything from the data pipeline
      // for that IMU. Hand position demands that position be enabled for all IMUs.
      if (DATA_LEGEND_FLAGS_IIU_REQ_POSITION != (per_iiu_data[i] & DATA_LEGEND_FLAGS_IIU_REQ_POSITION)) {
        per_iiu_data[i] = DATA_LEGEND_FLAGS_IIU_REQ_POSITION;
        return_value = true;
      }
    }
    else if (velocity(i)) {
      // Asking for velocity will require that we cancel gravity.
      if (DATA_LEGEND_FLAGS_IIU_REQ_VELOCITY != (per_iiu_data[i] & DATA_LEGEND_FLAGS_IIU_REQ_VELOCITY)) {
        per_iiu_data[i] = DATA_LEGEND_FLAGS_IIU_REQ_VELOCITY;
        return_value = true;
      }
    }
    else if (accNullGravity(i)) {
      // To cancel gravity, we need to know orientation.
      if (DATA_LEGEND_FLAGS_IIU_REQ_NULL_GRAV != (per_iiu_data[i] & DATA_LEGEND_FLAGS_IIU_REQ_NULL_GRAV)) {
        per_iiu_data[i] = DATA_LEGEND_FLAGS_IIU_REQ_NULL_GRAV;
        return_value = true;
      }
    }
    else if (orientation(i)) {
      // To find orientation, we need all the inertial data at minimum. Probably mag too.
      if (DATA_LEGEND_FLAGS_IIU_REQ_ORIENTATION != (per_iiu_data[i] & DATA_LEGEND_FLAGS_IIU_REQ_ORIENTATION)) {
        per_iiu_data[i] = DATA_LEGEND_FLAGS_IIU_REQ_ORIENTATION;
        return_value = true;
      }
    }
  }
  return return_value;
}



void ManuLegend::printManuLegend(StringBuilder* output) {
  output->concatf(
    "-- ManuLegend\n-----------------------------------\n");
  output->concatf("-- dataset_size   \t%u\n", (unsigned long) ds_size);
  output->concat("-- Enabled data:\n");
  output->concatf("\t handPosition   \t%c\n", handPosition() ? 'y' : 'n');
  output->concatf("\t Delta-T        \t%c\n", deltaT() ? 'y' : 'n');

  char* cap_str = (char*) alloca(13);
  *(cap_str+12) = 0;

  output->concat("\t          agmtoavpagmt\n\t          cyamrneossss\n\t          crgpiglscccc\n");
  for (uint8_t idx = 0; idx < LEGEND_DATASET_IIU_COUNT; idx++) {
    uint16_t d_opts = iiu_data_opts(idx);
    for (uint8_t bit = 0; bit < 12; bit++) {
      *(cap_str+bit) = (1 == ((d_opts >> bit) & 0x01)) ? '*' : ' ';
    }
    output->concatf("\t IIU %02u:  %s\n", idx, cap_str);
  }
}
