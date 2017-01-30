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
    if (update_rate_m != nu_srate_idx) {
      uint8_t temp8 = regValue(RegID::M_CTRL_REG1);
      if (0 == nu_srate_idx) {
        // Power the sensor down.
      }
      else {
        if (getVerbosity() > 2) Kernel::log("set_sample_rate_mag():\tMagnetometer sample rate change.\n");
        temp8 =  (temp8 & ~0x1C) | ((nu_srate_idx-1) << 2);
      }
      update_rate_m = nu_srate_idx;
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


/**
* When a bus operation completes, it is passed back to its issuing class.
*
* @param  idx    The register index that was updated.
* @param  value  The newest available value.
* @return IMUFault code.
*/
IMUFault LSM9DS1::io_op_callback_mag_read(RegID idx, unsigned int value) {
  IMUFault return_value = IMUFault::NO_ERROR;
  if (getVerbosity() > 6) local_log.concatf("io_op_callback_mag_read(%s, %u)\n", regNameString(idx), value);

  /* READ Case-offs */
  if (initPending()) {
    if (IDX_T1 == idx) {
      if (integrity_check()) {
        set_state(IMUState::STAGE_3);
        step_state();
      }
    }
  }

  switch (idx) {
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

    case RegID::M_CTRL_REG1:
      if (((value >> 2) & 0x07) < MAXIMUM_RATE_INDEX_MAG)  update_rate_m = ((value >> 6) & 0x03)+1;
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

  if (local_log.length() > 0) Kernel::log(&local_log);
  return return_value;
}


/**
* When a bus operation completes, it is passed back to its issuing class.
*
* @param  idx    The register index that was updated.
* @param  value  The newest available value.
* @return IMUFault code.
*/
IMUFault LSM9DS1::io_op_callback_mag_write(RegID idx, unsigned int value) {
  IMUFault return_value = IMUFault::NO_ERROR;
  if (getVerbosity() > 6) local_log.concatf("io_op_callback_mag_write(%s, %u)\n", regNameString(idx), value);

  /* WRITE Case-offs */
  if (initPending()) {
    if (IDX_T1 == idx) {
      set_state(IMUState::STAGE_2);
      step_state();
    }
  }

  switch (idx) {
    case RegID::M_CTRL_REG1:
      if (((value >> 2) & 0x07) < MAXIMUM_RATE_INDEX_MAG)  update_rate_m = ((value >> 6) & 0x03)+1;
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
      if (getVerbosity() > 5) local_log.concatf("\t M Wrote an unimplemented register.\n");
      break;
  }

  if (local_log.length() > 0) Kernel::log(&local_log);
  return return_value;
}
