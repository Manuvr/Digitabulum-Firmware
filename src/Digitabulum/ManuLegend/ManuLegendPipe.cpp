/*
File:   ManuLegendPipe.h
Author: J. Ian Lindsay
Date:   2017.08.18

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


The ManuLegendPipe is the means by which relevant manu data is filtered,
  encoded and sent towards a counterparty at a reasonable rate.
*/

#include <Kernel.h>
#include "ManuLegendPipe.h"
#include "SensorFrame.h"
#include <DataStructures/Argument.h>
#include "../CPLDDriver/CPLDDriver.h"

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

const char* ManuLegendPipe::encoding_label(ManuEncoding e) {
  switch (e) {
    case ManuEncoding::LOG:    return "LOG";
    case ManuEncoding::CBOR:   return "CBOR";
    case ManuEncoding::OSC:    return "OSC";
    case ManuEncoding::MANUVR: return "MANUVR";
  }
  return "";
};

const char* get_imu_label(int idx) {
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



/*******************************************************************************
*   ___ _              ___      _ _              _      _
*  / __| |__ _ ______ | _ ) ___(_) |___ _ _ _ __| |__ _| |_ ___
* | (__| / _` (_-<_-< | _ \/ _ \ | / -_) '_| '_ \ / _` |  _/ -_)
*  \___|_\__,_/__/__/ |___/\___/_|_\___|_| | .__/_\__,_|\__\___|
*                                          |_|
* Constructors/destructors, class initialization functions and so-forth...
*******************************************************************************/

/**
* Constructor
*/
ManuLegendPipe::ManuLegendPipe(ManuEncoding e) : ManuLegend(), BufferPipe() {
  stable(true);
  _encoding = e;
}

/**
* Destructor
*/
ManuLegendPipe::~ManuLegendPipe() {
}


/*******************************************************************************
*  _       _   _        _
* |_)    _|_ _|_ _  ._ |_) o ._   _
* |_) |_| |   | (/_ |  |   | |_) (/_
*                            |
* Overrides and addendums to BufferPipe.
*******************************************************************************/
const char* ManuLegendPipe::pipeName() { return "ManuLegendPipe"; }


void ManuLegendPipe::printDebug(StringBuilder* output) {
  output->concatf(
    "-- ManuLegendPipe  (%sactive %sstable)\n-----------------------------------\n",
    (active() ? "" : "in"), (stable() ? "" : "un")
  );
  BufferPipe::printDebug(output);
  output->concatf("-- Encoding       \t%s\n", ManuLegendPipe::encoding_label(_encoding));
  output->concatf("-- Legend Sent    \t%c\n", changeSent() ? 'y' : 'n');
  output->concatf("-- Data demands:  \t%satisfied\n", satisfied() ? "S" : "Uns");
  output->concatf("\t Sequence num   \t%c\n", sequence() ? 'y' : 'n');
}


void ManuLegendPipe::broadcast_legend() {
  StringBuilder* legend_string = new StringBuilder();
  getLegendString(legend_string);
  ManuvrMsg* legend_broadcast = Kernel::returnEvent(DIGITABULUM_MSG_IMU_LEGEND, _owner);
  legend_broadcast->specific_target = _owner;
  legend_broadcast->addArg(legend_string)->reapValue(true);
  Kernel::staticRaiseEvent(legend_broadcast);
  changeSent(true);
}


/**
* Calling this will cause the class to copy the data the owner requested from the
*   SensorFrame. We get data on a frame-by-frame basis, and we can assume that we only
*   have a lock on the data within this fxn scope. So anything indirected must be
*   copied here.
*
* @return non-zero on error.
*/
int8_t ManuLegendPipe::offer(SensorFrame* frame) {
  StringBuilder log;
  int8_t return_value = -1;
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
              output.concat(co.data(), final_size);
              log.concatf("CBOR frame: %d bytes\n", final_size);
              //log.printDebug(&output);
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
            log.concat("MANUVR output:\n");
            ret->serialize(&output);
            output.printDebug(&log);
            log.concat("\n");
            delete ret;
          }
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
          frame->printManuLegend(&log);
          return_value = 0;
          break;
      }
      _ms_last_send = millis() + _ms_interval;
      if (output.length()) {
        if (MEM_MGMT_RESPONSIBLE_ERROR != toCounterparty(&output, MEM_MGMT_RESPONSIBLE_BEARER)) {
          return_value = 0;
        }
        else {
          log.concat("ManuLegendPipe: Pipe failure.\n");
        }
      }
      else {
        log.concat("ManuLegendPipe: No output.\n");
      }
    }
    else {
      log.concatf("ManuLegendPipe: _ms_last_send = %u\n", _ms_last_send);
    }
  }
  else {
    log.concat("ManuLegendPipe: Not accepting frame.\n");
  }
  if (log.length()) {
    Kernel::log(&log);
  }
  return return_value;
}
