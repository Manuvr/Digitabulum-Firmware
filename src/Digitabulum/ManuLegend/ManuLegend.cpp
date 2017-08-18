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
#include <DataStructures/Argument.h>

/*******************************************************************************
*      _______.___________.    ___   .___________. __    ______     _______.
*     /       |           |   /   \  |           ||  |  /      |   /       |
*    |   (----`---|  |----`  /  ^  \ `---|  |----`|  | |  ,----'  |   (----`
*     \   \       |  |      /  /_\  \    |  |     |  | |  |        \   \
* .----)   |      |  |     /  _____  \   |  |     |  | |  `----.----)   |
* |_______/       |__|    /__/     \__\  |__|     |__|  \______|_______/
*
* Static members and initializers should be located here.
*******************************************************************************/

const char* const ManuLegend::encoding_label(ManuEncoding e) {
  switch (e) {
    case ManuEncoding::LOG:    return "LOG";
    case ManuEncoding::CBOR:   return "CBOR";
    case ManuEncoding::OSC:    return "OSC";
    case ManuEncoding::MANUVR: return "MANUVR";
  }
  return "";
};

const char* const get_imu_label(int idx) {
  switch (idx) {
    case 0:   return "c";
    case 1:   return "mc";
    case 2:   return "p1";
    case 3:   return "i1";
    case 4:   return "d1";
    case 5:   return "p2";
    case 6:   return "i2";
    case 7:   return "d2";
    case 8:   return "p3";
    case 9:   return "i3";
    case 10:  return "d3";
    case 11:  return "p4";
    case 12:  return "i4";
    case 13:  return "d4";
    case 14:  return "p5";
    case 15:  return "i5";
    case 16:  return "d5";
    default:  return "mistake";
  }
};


/****************************************************************************************************
*   ___ _              ___      _ _              _      _
*  / __| |__ _ ______ | _ ) ___(_) |___ _ _ _ __| |__ _| |_ ___
* | (__| / _` (_-<_-< | _ \/ _ \ | / -_) '_| '_ \ / _` |  _/ -_)
*  \___|_\__,_/__/__/ |___/\___/_|_\___|_| | .__/_\__,_|\__\___|
*                                          |_|
* Constructors/destructors, class initialization functions and so-forth...
****************************************************************************************************/

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
ManuLegend::ManuLegend(ManuEncoding e) {
  _encoding = e;
  // Make the Legend NULL at first. Nothing requested.
  for (uint8_t idx = 0; idx < LEGEND_DATASET_IIU_COUNT; idx++) per_iiu_data[idx] = 0;
}

/**
* Destructor
*/
ManuLegend::~ManuLegend() {
}


/**
* Calling this will cause the class to copy the data the owner requested from the
*   SensorFrame. We get data on a frame-by-frame basis, and we can assume that we only
*   have a lock on the data within this fxn scope. So anything indirected must be
*   copied here.
*
* @return non-zero on error.
*/
int8_t ManuLegend::offer(SensorFrame* frame) {
  if (should_accept()) {
    if (_ms_last_send <= millis()) {
      StringBuilder output;
      switch (_encoding) {
        case ManuEncoding::CBOR:
          {
            cbor::output_dynamic co;
            cbor::encoder encoder(co);
            if (sequence()) {
              encoder.write_map(1);
              encoder.write_string("seq");
              encoder.write_int(decoupleSeq() ? ++_local_seq : frame->seq());
            }
            if (deltaT()) {
              encoder.write_map(1);
              encoder.write_string("dt");
              encoder.write_float(frame->time());
            }
            if (handPosition()) {
              encoder.write_map(1);
              encoder.write_string("hp");
              encoder.write_tag(MANUVR_CBOR_VENDOR_TYPE | TcodeToInt(TCode::VECT_3_FLOAT));
              encoder.write_bytes((uint8_t*) &(frame->hand_position), 12);
            }
            for (uint8_t idx = 0; idx < LEGEND_DATASET_IIU_COUNT; idx++) {
              if (orientation(idx)) {
                encoder.write_map(1);
                encoder.write_string("ori");
                encoder.write_tag(MANUVR_CBOR_VENDOR_TYPE | TcodeToInt(TCode::VECT_4_FLOAT));
                encoder.write_bytes((uint8_t*) &(frame->quat[idx]), 16);
              }
              if (accNullGravity(idx)) {
                encoder.write_map(1);
                encoder.write_string("ang");
                encoder.write_tag(MANUVR_CBOR_VENDOR_TYPE | TcodeToInt(TCode::VECT_3_FLOAT));
                encoder.write_bytes((uint8_t*) &(frame->n_data[idx]), 12);
              }
              if (accRaw(idx)) {
                encoder.write_map(1);
                encoder.write_string("acc");
                encoder.write_tag(MANUVR_CBOR_VENDOR_TYPE | TcodeToInt(TCode::VECT_3_FLOAT));
                encoder.write_bytes((uint8_t*) &(frame->a_data[idx]), 12);
              }
              if (gyro(idx)) {
                encoder.write_map(1);
                encoder.write_string("gyr");
                encoder.write_tag(MANUVR_CBOR_VENDOR_TYPE | TcodeToInt(TCode::VECT_3_FLOAT));
                encoder.write_bytes((uint8_t*) &(frame->g_data[idx]), 12);
              }
              if (mag(idx)) {
                encoder.write_map(1);
                encoder.write_string("mag");
                encoder.write_tag(MANUVR_CBOR_VENDOR_TYPE | TcodeToInt(TCode::VECT_3_FLOAT));
                encoder.write_bytes((uint8_t*) &(frame->m_data[idx]), 12);
              }
              if (velocity(idx)) {
              }
              if (position(idx)) {
              }
              if (temperature(idx)) {
                encoder.write_map(1);
                encoder.write_string("tmp");
                encoder.write_float(frame->temperature[idx]);
              }
              if (samplesAcc(idx)) {
              }
              if (samplesGyro(idx)) {
              }
              if (samplesMag(idx)) {
              }
              if (samplesTemperature(idx)) {
              }
            }
            int final_size = co.size();
            if (final_size) {
              StringBuilder tmp_str(co.data(), final_size);
              output.concatf("CBOR frame: %d bytes\n", final_size);
              tmp_str.printDebug(&output);
              Kernel::log(&output);
            }
          }
          break;

        case ManuEncoding::MANUVR:
          {
            // TODO: This will be converted away from the heap-heavy Argument class
            //   once enough other pieces are talking again.
            Argument* ret = nullptr;
            if (sequence()) {
              Argument* nu = new Argument(decoupleSeq() ? ++_local_seq : frame->seq());
              nu->setKey("seq");
              if (ret) {
                ret->link(nu);
              }
              else {
                ret = nu;
              }
            }
            if (deltaT()) {
              Argument* nu = new Argument(frame->time());
              nu->setKey("dt");
              if (ret) {
                ret->link(nu);
              }
              else {
                ret = nu;
              }
            }
            if (handPosition()) {
              Argument* nu = new Argument(&(frame->hand_position));
              nu->setKey("hp");
              if (ret) {
                ret->link(nu);
              }
              else {
                ret = nu;
              }
            }

            for (uint8_t idx = 0; idx < LEGEND_DATASET_IIU_COUNT; idx++) {
              Argument* imu_arg = nullptr;
              if (orientation(idx)) {
              }
              if (accNullGravity(idx)) {
              }
              if (accRaw(idx)) {
              }
              if (gyro(idx)) {
              }
              if (mag(idx)) {
              }
              if (velocity(idx)) {
              }
              if (position(idx)) {
              }
              if (temperature(idx)) {
                Argument* nu = new Argument(frame->temperature[idx]);
                nu->setKey("temp");
                if (ret) {
                  ret->link(nu);
                }
                else {
                  ret = nu;
                }
              }
              if (samplesAcc(idx)) {
              }
              if (samplesGyro(idx)) {
              }
              if (samplesMag(idx)) {
              }
              if (samplesTemperature(idx)) {
              }
              if (imu_arg) {
                Argument* nu = new Argument(imu_arg);
                nu->setKey(get_imu_label(idx));
                if (ret) {
                  ret->link(nu);
                }
                else {
                  ret = nu;
                }
              }
            }
            StringBuilder temp;
            output.concat("MANUVR output:\n");
            ret->serialize(&temp);
            temp.printDebug(&output);
            output.concat("\n");
            delete ret;
          }
          Kernel::log(&output);
          break;

        case ManuEncoding::OSC:
          {
            if (sequence()) {
            }
            if (deltaT()) {
            }
            if (handPosition()) {
            }
            for (uint8_t idx = 0; idx < LEGEND_DATASET_IIU_COUNT; idx++) {
              if (orientation(idx)) {
              }
              if (accNullGravity(idx)) {
              }
              if (accRaw(idx)) {
              }
              if (gyro(idx)) {
              }
              if (mag(idx)) {
              }
              if (velocity(idx)) {
              }
              if (position(idx)) {
              }
              if (temperature(idx)) {
              }
              if (samplesAcc(idx)) {
              }
              if (samplesGyro(idx)) {
              }
              if (samplesMag(idx)) {
              }
              if (samplesTemperature(idx)) {
              }
            }
          }
          break;
        case ManuEncoding::LOG:
          frame->printDebug(&output);
          Kernel::log(&output);
          break;
      }
      _ms_last_send = millis() + _ms_interval;
    }
  }
  return 0;
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


void ManuLegend::printDebug(StringBuilder *output) {
  output->concatf(
    "-- ManuLegend  (%sactive %sstable)\n-----------------------------------\n",
    (active() ? "" : "in"), (stable() ? "" : "un")
  );
  output->concatf("-- Encoding       \t%s\n", ManuLegend::encoding_label(_encoding));
  output->concatf("-- dataset_size   \t%u\n", (unsigned long) ds_size);
  output->concatf("-- Enabled data:  (%satisfied)\n", satisfied() ? "S" : "Uns");
  output->concatf("\t Sequence num   \t%c\n", sequence() ? 'y' : 'n');
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
