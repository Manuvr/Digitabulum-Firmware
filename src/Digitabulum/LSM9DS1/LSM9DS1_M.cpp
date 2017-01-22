/*
File:   LSM9DS1_M.cpp
Author: J. Ian Lindsay
Date:   2014.03.27

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

#include "LSM9DS1.h"
#include "../ManuLegend/ManuManager.h"

/*******************************************************************************
*  __  __                        _                       _
* |  \/  |                      | |                     | |
* | \  / | __ _  __ _ _ __   ___| |_ ___  _ __ ___   ___| |_ ___ _ __
* | |\/| |/ _` |/ _` | '_ \ / _ \ __/ _ \| '_ ` _ \ / _ \ __/ _ \ '__|
* | |  | | (_| | (_| | | | |  __/ || (_) | | | | | |  __/ ||  __/ |
* |_|  |_|\__,_|\__, |_| |_|\___|\__\___/|_| |_| |_|\___|\__\___|_|
*                __/ |
*               |___/
*******************************************************************************/
/*
* Magnetometer data
* Tests all the required registers for freshness and builds a Vector with the new reading
*   if possible.
* Returns...
*   0 on success with no vector.
*   1 on success with new vector.
*  -1 on error.
*/
int8_t LSM9DS1::collect_reading_mag() {
  /* Ok... so we know that if we got here, the pre-formed bus op is freshly execed,
       so we are going to grab its ending timestamp and populate the measurement's
       field with the value corrected for boot-time.
     TODO: This will have to revisited later to account for clock-wrap in a clean manner.
       Until then, it will cause us strange bugs, and possibly even a crash at the wrap.
  */

  Vector3<int16_t> reflection_vector_mag(ManuManager::reflection_mag.x, ManuManager::reflection_mag.y, ManuManager::reflection_mag.z);

  float scaler = error_map_mag[scale_mag].per_lsb;
  //float x = ((((int16_t)regValue(RegID::AG_DATA_X_M) - noise_floor_mag_mag.x) * reflection_vector_mag.x) * scaler);
  //float y = ((((int16_t)regValue(RegID::AG_DATA_Y_M) - noise_floor_mag_mag.y) * reflection_vector_mag.y) * scaler);
  //float z = ((((int16_t)regValue(RegID::AG_DATA_Z_M) - noise_floor_mag_mag.z) * reflection_vector_mag.z) * scaler);
  float x = ((((int16_t)regValue(RegID::M_DATA_X)) * reflection_vector_mag.x) * scaler);
  float y = ((((int16_t)regValue(RegID::M_DATA_Y)) * reflection_vector_mag.y) * scaler);
  float z = ((((int16_t)regValue(RegID::M_DATA_Z)) * reflection_vector_mag.z) * scaler);

  last_val_mag(x, y, z);
  //integrator->pushMeasurement(IMU_FLAG_MAG_DATA, x, y, z, rate_settings_mag[update_rate_mag].ts_delta);

  return 1;
}



/**
* Call to rescale the sensor.
*/
IMUFault LSM9DS1::request_rescale_mag(uint8_t nu_scale_idx) {
  if (nu_scale_idx < MAXIMUM_GAIN_INDEX_MAG) {
    if (scale_mag != nu_scale_idx) {
      if (getVerbosity() > 2) Kernel::log("request_rescale_mag():\tRescaling magnetometer.\n");
      return writeRegister(RegID::M_CTRL_REG2, (nu_scale_idx << 5));
    }
    return IMUFault::NO_ERROR;
  }
  return IMUFault::INVALID_PARAM_ID;
}


/**
* Call to alter sample rate.
*/
IMUFault LSM9DS1::set_sample_rate_mag(uint8_t nu_srate_idx) {
  if (nu_srate_idx < MAXIMUM_RATE_INDEX_MAG) {
    if (update_rate_mag != nu_srate_idx) {
      uint8_t temp8 = regValue(RegID::M_CTRL_REG1);
      if (0 == nu_srate_idx) {
        // Power the sensor down.
      }
      else {
        if (getVerbosity() > 2) Kernel::log("set_sample_rate_mag():\tMagnetometer sample rate change.\n");
        temp8 =  (temp8 & ~0x1C) | ((nu_srate_idx-1) << 2);
      }
      update_rate_mag = nu_srate_idx;
      return writeRegister(RegID::M_CTRL_REG1, temp8);
    }
    return IMUFault::NO_ERROR;
  }
  return IMUFault::INVALID_PARAM_ID;
}



IMUFault LSM9DS1::irq_drdy() {
  if (getVerbosity() > 3) Kernel::log("LSM9DS1::irq_drdy()\n");
  return IMUFault::NO_ERROR;
}


IMUFault LSM9DS1::irq_m() {
  if (getVerbosity() > 3) Kernel::log("LSM9DS1::irq_m()\n");
  return IMUFault::NO_ERROR;
}


int8_t LSM9DS1::calibrate_from_data_mag() {
  // Is reading stable?
  // Average vectors....
  Vector3<int32_t> avg;
  for (int i = 0; i < 32; i++) {
    avg.x += sample_backlog_mag[i].x;
    avg.y += sample_backlog_mag[i].y;
    avg.z += sample_backlog_mag[i].z;
  }
  avg /= 32;
  noise_floor_mag.x = (int16_t) avg.x;
  noise_floor_mag.y = (int16_t) avg.y;
  noise_floor_mag.z = (int16_t) avg.z;
  return 0;
}


/**
* When a bus operation completes, it is passed back to its issuing class.
*
* @param  _op  The bus operation that was completed.
* @return SPI_CALLBACK_NOMINAL on success, or appropriate error code.
*/
int8_t LSM9DS1::io_op_callback_mag(SPIBusOp* op) {
  int8_t return_value = SPI_CALLBACK_NOMINAL;

  unsigned int access_len = op->buf_len;  // The access length lets us know how many things changed.
  uint8_t access_idx = op->getTransferParam(3);
  unsigned int value = regValue(access_idx);
  if (getVerbosity() > 6) local_log.concatf("%s  G::io_op_callback(%p): value: %d \t access_idx  %d \t access_len: %d\n", op->getOpcodeString(), (uintptr_t)((BusOpCallback*) this), value, access_idx, access_len);

  /* Our first choice is: Did we just finish a WRITE or a READ? */
  /* READ Case-offs */
  if (BusOpcode::RX == op->get_opcode()) {
    while (access_len > 0) {
      value = regValue(access_idx);
      access_len -= 1;   // Subtract the length.

      if (getVerbosity() > 3) {
        local_log.concatf("\t GY R: access_idx  0x%02x   (0x%04x)\n", access_idx, (uint16_t) value);
      }

      if (initPending()) {
        if (IDX_T1 == access_idx) {
          if (integrity_check()) {
            set_state(IMUState::STAGE_3);
            step_state();
          }
        }
      }

      switch (access_idx) {
        case RegID::M_WHO_AM_I:
          if (0x3D == value) {
            if (!present()) {
              set_state(IMUState::STAGE_1);
              step_state();
            }
          }
          else {
            // We lost the IMU, perhaps...
            set_state(IMUState::STAGE_0);
            error_condition = IMUFault::WRONG_IDENTITY;
          }
          break;

        case RegID::M_DATA_X:
          collect_reading_mag();
        /* We don't address these registers byte-wise. Empty case for documentation's sake. */
        case RegID::M_DATA_Y:  break;
        case RegID::M_DATA_Z:  break;


        case RegID::M_CTRL_REG1:
          if (((value >> 2) & 0x07) < MAXIMUM_RATE_INDEX_MAG)  update_rate_mag = ((value >> 6) & 0x03)+1;
          break;

        case RegID::M_CTRL_REG2:
          if (((value >> 4) & 0x03) < MAXIMUM_GAIN_INDEX_MAG)  scale_mag = (value >> 5) & 0x03;
          break;

        case RegID::M_CTRL_REG3:
          power_to_mag(value & 0x02);
          break;

        case RegID::M_CTRL_REG4:
          break;

        case RegID::M_CTRL_REG5:
          break;

        default:
          break;
      }
      if (op->devRegisterAdvance()) access_idx++;
    }
  }


  /* WRITE Case-offs */
  else if (BusOpcode::TX == op->get_opcode()) {
    while (access_len > 0) {
      value = regValue(access_idx);
      access_len -= 1;   // Subtract the length.
      if (getVerbosity() > 3) {
        local_log.concatf("\t G  W: access_idx  0x%02x   (0x%04x)\n", access_idx, (uint16_t) value);
      }

      if (initPending()) {
        if (IDX_T1 == access_idx) {
          set_state(IMUState::STAGE_2);
          step_state();
        }
      }

      switch (access_idx) {
        case RegID::M_CTRL_REG1:
          if (((value >> 2) & 0x07) < MAXIMUM_RATE_INDEX_MAG)  update_rate_mag = ((value >> 6) & 0x03)+1;
          break;

        case RegID::M_CTRL_REG2:
          if (((value >> 4) & 0x03) < MAXIMUM_GAIN_INDEX_MAG)  scale_mag = (value >> 5) & 0x03;
          if (value & 0x04) { // Did we write here to reset?
            if (!present()) {
              //integrator->init();
            }
          }
          break;

        case RegID::M_CTRL_REG3:
          power_to_mag(value & 0x02);
          break;

        case RegID::M_CTRL_REG5:
          break;

        default:
          if (getVerbosity() > 5) local_log.concatf("\t G Wrote an unimplemented register.\n");
          break;
      }
      if (op->devRegisterAdvance()) access_idx++;
    }
  }

  if (local_log.length() > 0) Kernel::log(&local_log);
  return return_value;
}
