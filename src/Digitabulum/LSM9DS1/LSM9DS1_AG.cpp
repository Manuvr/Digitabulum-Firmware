/*
File:   LSM9DS1_AG.cpp
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
*                       _                               _
*     /\               | |                             | |
*    /  \   ___ ___ ___| | ___ _ __ ___  _ __ ___   ___| |_ ___ _ __
*   / /\ \ / __/ __/ _ \ |/ _ \ '__/ _ \| '_ ` _ \ / _ \ __/ _ \ '__|
*  / ____ \ (_| (_|  __/ |  __/ | | (_) | | | | | |  __/ ||  __/ |
* /_/    \_\___\___\___|_|\___|_|  \___/|_| |_| |_|\___|\__\___|_|
*
*******************************************************************************/


/**
* Call to rescale the sensor.
*/
IMUFault LSM9DS1::request_rescale_acc(uint8_t nu_scale_idx) {
  if (nu_scale_idx < MAXIMUM_GAIN_INDEX_ACC) {
    if (scale_acc != nu_scale_idx) {
      uint8_t temp8 = regValue(RegID::A_CTRL_REG6);
      temp8 =  (temp8 & 0xE7) | (nu_scale_idx << 3);
      return writeRegister(RegID::A_CTRL_REG6, temp8);
    }
    return IMUFault::NO_ERROR;
  }
  return IMUFault::INVALID_PARAM_ID;
}


/**
* Call to alter sample rate.
*/
IMUFault LSM9DS1::set_sample_rate_acc(uint8_t nu_srate_idx) {
  if (nu_srate_idx < MAXIMUM_RATE_INDEX_AG) {
    if (update_rate_i != nu_srate_idx) {
      uint8_t temp8 = regValue(RegID::A_CTRL_REG6);
      temp8 =  (temp8 & 0x1F) | (nu_srate_idx << 5);
      update_rate_i = nu_srate_idx;
      return writeRegister(RegID::A_CTRL_REG6, temp8);
    }
    return IMUFault::NO_ERROR;
  }
  return IMUFault::INVALID_PARAM_ID;
}


// Call to change the bandwidth of the AA filter.
IMUFault LSM9DS1::set_base_filter_param_acc(uint8_t nu_bw_idx) {
  if ((nu_bw_idx < 4) && (nu_bw_idx != base_filter_param)) {
    base_filter_param = nu_bw_idx;
    uint8_t temp8 = regValue(RegID::A_CTRL_REG6);
    temp8 =  (temp8 & 0xFC) | (nu_bw_idx & 0x03);
    return writeRegister(RegID::A_CTRL_REG6, temp8);
  }
  return IMUFault::INVALID_PARAM_ID;
}



/*******************************************************************************
*   _____
*  / ____|
* | |  __ _   _ _ __ ___
* | | |_ | | | | '__/ _ \
* | |__| | |_| | | | (_) |
*  \_____|\__, |_|  \___/
*          __/ |
*         |___/
*******************************************************************************/

/**
* Call to rescale the sensor.
*/
IMUFault LSM9DS1::request_rescale_gyr(uint8_t nu_scale_idx) {
  if (nu_scale_idx < MAXIMUM_GAIN_INDEX_ACC) {
    if (scale_acc != nu_scale_idx) {
      if (getVerbosity() > 2) Kernel::log("request_rescale_gyr():\tRescaling Gyro.\n");
      uint8_t temp8 = regValue(RegID::G_CTRL_REG1);
      temp8 =  (temp8 & 0xE7) | (nu_scale_idx << 3);
      return writeRegister(RegID::G_CTRL_REG1, temp8);
    }
    return IMUFault::NO_ERROR;
  }
  return IMUFault::INVALID_PARAM_ID;
}


/**
* Call to alter sample rate.
*/
IMUFault LSM9DS1::set_sample_rate_gyr(uint8_t nu_srate_idx) {
  if (nu_srate_idx < MAXIMUM_RATE_INDEX_AG) {
    if (update_rate_i != nu_srate_idx) {
      if (getVerbosity() > 2) Kernel::log("set_sample_rate_gyr():\t\n");
      uint8_t temp8 = regValue(RegID::G_CTRL_REG1);
      temp8 =  (temp8 & 0x1F) | (nu_srate_idx << 5);
      //update_rate_gyr = nu_srate_idx;
      return writeRegister(RegID::G_CTRL_REG1, temp8);
    }
    return IMUFault::NO_ERROR;
  }
  return IMUFault::INVALID_PARAM_ID;
}



// Call to change the cutoff of the gyro's base filter.
IMUFault LSM9DS1::set_base_filter_param_gyr(uint8_t nu_bw_idx) {
  return IMUFault::INVALID_PARAM_ID;
}



/*
* The purpose here is to figure out why one of our interrupt lines is going off.
* Because this sensor is so awesome, it has configurable interrupt sources that
*   are bindable to two separate pins.
* Since everything is async, we will process this task in stages. We need to
*   keep in mind that there will be two other devices sharing IRQ lines with
*   this one, and we need to give them a chance to read their registers before
*   their interrupts go stale.
// TODO: staggered priorities if separate bus access? Might help find the cause faster...

* Stage 0) Read all relevant registers. Return.
* Stage 1) Process register data and service interrupt if applicable in io_op_callback().
*
* Return codes:
*   Success, with IRQ service
*   Success, no IRQ service
*   Failure
*/
IMUFault LSM9DS1::irq_2() {
  if (getVerbosity() > 3) Kernel::log("LSM9DS1::irq_2()\n");
  return IMUFault::NO_ERROR;
}

IMUFault LSM9DS1::irq_1() {
  if (getVerbosity() > 3) Kernel::log("LSM9DS1::irq_1()\n");
  return IMUFault::NO_ERROR;
}


/**
* When a bus operation completes, it is passed back to its issuing class.
*
* @param  idx    The register index that was updated.
* @param  value  The newest available value.
* @return IMUFault code.
*/
IMUFault LSM9DS1::io_op_callback_ag_read(RegID idx, unsigned int value) {
  IMUFault return_value = IMUFault::NO_ERROR;
  if (getVerbosity() > 6) local_log.concatf("io_op_callback_ag_read(%s, %u)\n", regNameString(idx), value);

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
    case RegID::A_DATA_X:
    case RegID::A_DATA_Y:
    case RegID::A_DATA_Z:
      break;

    case RegID::G_DATA_X:
    case RegID::G_DATA_Y:
    case RegID::G_DATA_Z:
      break;

    case RegID::AG_WHO_AM_I:
      if (0x68 == value) {
        if (!present()) {
          set_state(IMUState::STAGE_1);
          step_state();
        }
        else {
          // Nominal condition. Maybe we bulk-read? Nice to have verification....
        }
      }
      else {
        // We lost the IMU, perhaps...
        set_state(IMUState::STAGE_0);
        error_condition = IMUFault::WRONG_IDENTITY;
      }
      break;

    case RegID::G_INT_GEN_SRC:     /* The gyroscope interrupt status register. */
      if (value & 0x01) {                 // An interrupt was seen because we crossed a threshold we set.
        if (value & 0xE0) {               // Did we exceed our set threshold?
          if (autoscale_gyr()) request_rescale_gyr(scale_gyr+1);
        }
        else if (value & 0x1C) {          // Did we drop below our set threshold?
          if (autoscale_gyr()) request_rescale_gyr(scale_gyr-1);
        }
      }
      else if (value & 0x02) {            // We had a range overflow. Means we need to autoscale...
        if (autoscale_gyr()) request_rescale_gyr(scale_gyr+1);
      }
      break;
    // TODO: We need to implement these....
    case RegID::AG_STATUS_REG:      /* Status of the gyr data registers on the sensor. */
      break;

    case RegID::AG_STATUS_REG_ALT:
      if (getVerbosity() > 5) local_log.concatf("\t RegID::AG_STATUS_REG_ALT: 0x%02x\n", (uint8_t) value);
      break;
    case RegID::AG_FIFO_CTRL:
      if (getVerbosity() > 5) local_log.concatf("\t AG_FIFO Control: 0x%02x\n", (uint8_t) value);
      break;
    case RegID::G_CTRL_REG1:
      if (getVerbosity() > 5) local_log.concatf("\t RegID::G_CTRL_REG1: 0x%02x\n", (uint8_t) value);
      if ((value >> 4) < MAXIMUM_RATE_INDEX_AG)  update_rate_i = (value >> 4) & 0x0F;
      break;
    case RegID::AG_CTRL_REG4:
      if (getVerbosity() > 5) local_log.concatf("\t RegID::AG_CTRL_REG4: 0x%02x\n", (uint8_t) value);
      if (((value >> 3) & 0x07) < MAXIMUM_GAIN_INDEX_ACC)  scale_acc = (value >> 3) & 0x07;
      base_filter_param = (value >> 6) & 0x03;
      break;
    case RegID::A_CTRL_REG6:
      if (getVerbosity() > 5) local_log.concatf("\t RegID::A_CTRL_REG6: 0x%02x\n", (uint8_t) value);
      if (((value >> 3) & 0x03) < MAXIMUM_GAIN_INDEX_GYR)  scale_gyr = (value >> 3) & 0x03;
      break;
    case RegID::AG_FIFO_SRC:     /* The FIFO status register. */
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
IMUFault LSM9DS1::io_op_callback_ag_write(RegID idx, unsigned int value) {
  IMUFault return_value = IMUFault::NO_ERROR;
  if (getVerbosity() > 6) local_log.concatf("io_op_callback_ag_write(%s, %u)\n", regNameString(idx), value);

  /* WRITE Case-offs */
  if (initPending()) {
    if (IDX_T1 == idx) {
      set_state(IMUState::STAGE_2);
      step_state();
    }
  }

  switch (idx) {
    // TODO: We need to implement these....
    case RegID::AG_INT1_CTRL:
      break;
    case RegID::AG_INT2_CTRL:
      break;

    case RegID::G_CTRL_REG1:
      //if ((value >> 4) < MAXIMUM_RATE_INDEX_AG)  update_rate_i = (value >> 4) & 0x0F;
      break;
    case RegID::G_CTRL_REG2:
      if (((value >> 3) & 0x07) < MAXIMUM_GAIN_INDEX_ACC)  scale_acc = (value >> 3) & 0x07;
      break;
    case RegID::A_CTRL_REG5:
      //if (((value >> 2) & 0x07) < MAXIMUM_RATE_INDEX_MAG)  update_rate_mag = ((value >> 2) & 0x07) + 1;
      break;
    case RegID::A_CTRL_REG6:
      if (((value >> 5) & 0x03) < MAXIMUM_GAIN_INDEX_GYR)  scale_gyr = (value >> 5) & 0x03;
      break;
    case RegID::AG_CTRL_REG8:
      if (value & 0x01) { // Did we write here to reset?
      }
      break;

    default:
      if (getVerbosity() > 5) local_log.concatf("\t AG Wrote an unimplemented register.\n");
      break;
  }

  if (local_log.length() > 0) Kernel::log(&local_log);
  return return_value;
}
