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
/*
* Accelerometer data
* Tests all the required registers for freshness and builds a Vector with the new reading
*   if possible.
* Returns...
*   0 on success with no vector.
*   1 on success with new vector.
*  -1 on error.
*/
int8_t LSM9DS1::collect_reading_acc() {
  /* Ok... so we know that if we got here, the pre-formed bus op is freshly execed,
       so we are going to grab it's ending timestamp and populate the measurement's
       field with the value corrected for boot-time.
     TODO: This will have to revisited later to account for clock-wrap in a clean manner.
       Until then, it will cause us strange bug, and possibly even a crash at the wrap.
  */
  sample_count++;

  float scaler = error_map_acc[scale_acc].per_lsb;
  float x;
  float y;
  float z;

  Vector3<int16_t> reflection_vector_acc(ManuManager::reflection_acc.x, ManuManager::reflection_acc.y, ManuManager::reflection_acc.z);

  if (cancel_error()) {
    x = ((((int16_t)regValue(LSM9DS1_A_DATA_X) - noise_floor_acc.x) * reflection_vector_acc.x) * scaler);
    y = ((((int16_t)regValue(LSM9DS1_A_DATA_Y) - noise_floor_acc.y) * reflection_vector_acc.y) * scaler);
    z = ((((int16_t)regValue(LSM9DS1_A_DATA_Z) - noise_floor_acc.z) * reflection_vector_acc.z) * scaler);
  }
  else {
    x = (((int16_t)regValue(LSM9DS1_A_DATA_X)) * reflection_vector_acc.x * scaler);
    y = (((int16_t)regValue(LSM9DS1_A_DATA_Y)) * reflection_vector_acc.y * scaler);
    z = (((int16_t)regValue(LSM9DS1_A_DATA_Z)) * reflection_vector_acc.z * scaler);
  }

  last_val_acc(x, y, z);
  integrator->pushMeasurement(IMU_FLAG_ACCEL_DATA, x, y, z, rate_settings_acc[update_rate_acc].ts_delta);

  return 1;
}


/**
* Call to rescale the sensor.
*/
int8_t LSM9DS1::request_rescale_acc(uint8_t nu_scale_idx) {
  if (nu_scale_idx < MAXIMUM_GAIN_INDEX_ACC) {
    if (scale_acc != nu_scale_idx) {
      uint8_t temp8 = regValue(LSM9DS1_A_CTRL_REG6);
      temp8 =  (temp8 & 0xE7) | (nu_scale_idx << 3);
      return writeRegister(LSM9DS1_A_CTRL_REG6, temp8);
    }
    return -2;
  }
  return -1;
}


/**
* Call to alter sample rate.
*/
int8_t LSM9DS1::set_sample_rate_acc(uint8_t nu_srate_idx) {
  if (nu_srate_idx < MAXIMUM_RATE_INDEX_AG) {
    if (update_rate_acc != nu_srate_idx) {
      uint8_t temp8 = regValue(LSM9DS1_A_CTRL_REG6);
      temp8 =  (temp8 & 0x1F) | (nu_srate_idx << 5);
      update_rate_acc = nu_srate_idx;
      return writeRegister(LSM9DS1_A_CTRL_REG6, temp8);
    }
    return -2;
  }
  return -1;
}


// Call to change the bandwidth of the AA filter.
int8_t LSM9DS1::set_base_filter_param_acc(uint8_t nu_bw_idx) {
  if ((nu_bw_idx < 4) && (nu_bw_idx != base_filter_param)) {
    base_filter_param = nu_bw_idx;
    uint8_t temp8 = regValue(LSM9DS1_A_CTRL_REG6);
    temp8 =  (temp8 & 0xFC) | (nu_bw_idx & 0x03);
    return writeRegister(LSM9DS1_A_CTRL_REG6, temp8);
  }
  return -1;
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

/*
* Gyroscope data
* Tests all the required registers for freshness and builds a Vector with the new reading
*   if possible.
* Returns...
*   0 on success with no vector.
*   1 on success with new vector.
*  -1 on error.
*/
int8_t LSM9DS1::collect_reading_gyr() {
  /* Ok... so we know that if we got here, the pre-formed bus op is freshly execed,
       so we are going to grab it's ending timestamp and populate the measurement's
       field with the value corrected for boot-time.
     TODO: This will have to revisited later to account for clock-wrap in a clean manner.
       Until then, it will cause us strange bug, and possibly even a crash at the wrap.
  */
  sample_count++;

  float scaler = error_map_gyr[scale_gyr].per_lsb;
  float x;
  float y;
  float z;

  Vector3<int16_t> reflection_vector_gyr(ManuManager::reflection_gyr.x, ManuManager::reflection_gyr.y, ManuManager::reflection_gyr.z);

  if (cancel_error()) {
    x = ((((int16_t)regValue(LSM9DS1_G_DATA_X)) - noise_floor_gyr.x) * reflection_vector_gyr.x * scaler);
    y = ((((int16_t)regValue(LSM9DS1_G_DATA_Y)) - noise_floor_gyr.y) * reflection_vector_gyr.y * scaler);
    z = ((((int16_t)regValue(LSM9DS1_G_DATA_Z)) - noise_floor_gyr.z) * reflection_vector_gyr.z * scaler);
  }
  else {
    x = ((((int16_t)regValue(LSM9DS1_G_DATA_X)) * reflection_vector_gyr.x) * scaler);
    y = ((((int16_t)regValue(LSM9DS1_G_DATA_Y)) * reflection_vector_gyr.y) * scaler);
    z = ((((int16_t)regValue(LSM9DS1_G_DATA_Z)) * reflection_vector_gyr.z) * scaler);
  }

  last_val_gyr(x, y, z);
  integrator->pushMeasurement(IMU_FLAG_GYRO_DATA, x, y, z, rate_settings_gyr[update_rate_gyr].ts_delta);
  return 1;
}


/**
* Call to rescale the sensor.
*/
int8_t LSM9DS1::request_rescale_gyr(uint8_t nu_scale_idx) {
  if (nu_scale_idx < MAXIMUM_GAIN_INDEX_ACC) {
    if (scale_acc != nu_scale_idx) {
      if (getVerbosity() > 2) Kernel::log("request_rescale_gyr():\tRescaling Gyro.\n");
      uint8_t temp8 = regValue(LSM9DS1_G_CTRL_REG1);
      temp8 =  (temp8 & 0xE7) | (nu_scale_idx << 3);
      return writeRegister(LSM9DS1_G_CTRL_REG1, temp8);
    }
    return -2;
  }
  return -1;
}


/**
* Call to alter sample rate.
*/
int8_t LSM9DS1::set_sample_rate_gyr(uint8_t nu_srate_idx) {
  if (nu_srate_idx < MAXIMUM_RATE_INDEX_AG) {
    if (update_rate_acc != nu_srate_idx) {
      if (getVerbosity() > 2) Kernel::log("set_sample_rate_gyr():\t\n");
      uint8_t temp8 = regValue(LSM9DS1_G_CTRL_REG1);
      temp8 =  (temp8 & 0x1F) | (nu_srate_idx << 5);
      update_rate_gyr = nu_srate_idx;
      return writeRegister(LSM9DS1_G_CTRL_REG1, temp8);
    }
    return -2;
  }
  return -1;
}



// Call to change the cutoff of the gyro's base filter.
int8_t LSM9DS1::set_base_filter_param_gyr(uint8_t nu_bw_idx) {
  return -1;
}


/*
* Temperature data
* Tests all the required registers for freshness and updates the integrator with the temperature
*   if possible.
* Returns...
*   0 on success with no changes made.
*   1 on success with new temperature point.
*  -1 on error.
*/
int8_t LSM9DS1::collect_reading_temperature() {
  int8_t return_value = 1;
  integrator->setTemperature(((int16_t)regValue(LSM9DS1_AG_DATA_TEMP)) / 1.0f);
  //integrator->setTemperature(((int16_t) value) / 4.0f);
  return return_value;
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
int8_t LSM9DS1::irq_0() {
  int8_t return_value = IMU_ERROR_NO_ERROR;

  if (getVerbosity() > 3) Kernel::log("XM::irq_0()\n");
  if (initComplete()) {
    if (!fire_preformed_bus_op(&preformed_busop_irq_0) ) {
      // Take corrective action.
    }
  }
  return return_value;
}

int8_t LSM9DS1::irq_1() {
  int8_t return_value = IMU_ERROR_NO_ERROR;

  if (getVerbosity() > 3) Kernel::log("XM::irq_1()\n");
  if (initComplete()) {
    //readRegister((uint8_t) LSM9DS1_AG_STATUS_REG_M);
    //readRegister((uint8_t) LSM9DS1_AG_FIFO_SRC_REG);
    if (!fire_preformed_bus_op(&preformed_busop_irq_1) ) {
      // Take corrective action.
    }
  }
  return return_value;
}


int8_t LSM9DS1::calibrate_from_data_ag() {
  // Average vectors....
  Vector3<int32_t> avg;
  for (int i = 0; i < 32; i++) {
    avg.x += sample_backlog_acc[i].x;
    avg.y += sample_backlog_acc[i].y;
    avg.z += sample_backlog_acc[i].z;
  }
  avg /= 32;   // This is our idea of gravity.
  noise_floor_acc.x = (int16_t) avg.x;
  noise_floor_acc.y = (int16_t) avg.y;
  noise_floor_acc.z = (int16_t) avg.z;

  for (int i = 0; i < 32; i++) {
    avg.x += sample_backlog_gyr[i].x;
    avg.y += sample_backlog_gyr[i].y;
    avg.z += sample_backlog_gyr[i].z;
  }
  avg /= 32;   // This is our idea of gravity.
  noise_floor_gyr.x = (int16_t) avg.x;
  noise_floor_gyr.y = (int16_t) avg.y;
  noise_floor_gyr.z = (int16_t) avg.z;

  float scaler = error_map_acc[scale_acc].per_lsb;
  float x = ((noise_floor_acc.x) * scaler);
  float y = ((noise_floor_acc.y) * scaler);
  float z = ((noise_floor_acc.z) * scaler);

  integrator->pushMeasurement(IMU_FLAG_GRAVITY_DATA, x, y, z, 0);

  // This is our idea of magentic North. Write it to the offset registers.
  //for (int i = 0; i < 8; i++) {
  //  *(register_pool + 14 + i) = *(register_pool + 3 + i);
  //}
  //writeRegister(LSM9DS1_AG_INT_THS_L_M, (register_pool + 12),   8, true);
  scaler = error_map_gyr[scale_gyr].per_lsb;
  Vector3<int16_t> reflection_vector_gyr(ManuManager::reflection_gyr.x, ManuManager::reflection_gyr.y, ManuManager::reflection_gyr.z);

  x = ((int16_t) regValue(LSM9DS1_G_DATA_X)) * scaler * reflection_vector_gyr.x;
  y = ((int16_t) regValue(LSM9DS1_G_DATA_Y)) * scaler * reflection_vector_gyr.y;
  z = ((int16_t) regValue(LSM9DS1_G_DATA_Z)) * scaler * reflection_vector_gyr.z;

  //Vector3<float> proj_xy(x, y, 0);
  //Vector3<float> proj_xz(x, 0, z);
  //Vector3<float> x_axis(1.0f, 0.0f, 0.0f);
  //
  //float offset_angle_y = Vector3<float>::angle(&proj_xy, &x_axis);
  //float offset_angle_z = Vector3<float>::angle(&proj_xz, &x_axis);

  //integrator->pushMeasurement(IMU_FLAG_BEARING_DATA, 0, offset_angle_y, offset_angle_z, 0);
  integrator->pushMeasurement(IMU_FLAG_BEARING_DATA, x, y, z, 0);

  return 0;
}


/**
* When a bus operation completes, it is passed back to its issuing class.
*
* @param  _op  The bus operation that was completed.
* @return SPI_CALLBACK_NOMINAL on success, or appropriate error code.
*/
int8_t LSM9DS1::io_op_callback_ag(SPIBusOp* op) {
  int8_t return_value = SPI_CALLBACK_NOMINAL;

  unsigned int access_len = op->buf_len;  // The access length lets us know how many things changed.
  uint8_t access_idx = op->getTransferParam(3);
  unsigned int value = regValue(access_idx);
  if (getVerbosity() > 6) local_log.concatf("%s  XM::io_op_callback(%p): value: %d \t access_idx  %d \t access_len: %d\n", op->getOpcodeString(), (uintptr_t)((BusOpCallback*) this), value, access_idx, access_len);

  /* Our first choice is: Did we just finish a WRITE or a READ? */
  /* READ Case-offs */
  if (BusOpcode::RX == op->get_opcode()) {
    while ((access_len > 0) && regExists(access_idx)) {
      value = regValue(access_idx);
      access_len -= 1;   // Subtract the length.

      if (getVerbosity() > 3) {
        local_log.concatf("\t XM R: access_idx  0x%02x   (0x%04x)\n", access_idx, (uint16_t) value);
      }

      if (initPending()) {
        if (IDX_T1 == access_idx) {
          if (integrity_check()) {
            set_state(State::STAGE_3);
            if (step_state()) {
              integrator->init();
            }
          }
        }
      }


      switch (access_idx) {
        case LSM9DS1_A_DATA_X:
          if (pending_samples > 0) {
            pending_samples--;
            switch (getState()) {
              case State::STAGE_5:
                collect_reading_acc();
                if (0 == pending_samples) {
                  set_state(State::STAGE_4);
                  step_state();
                }
                else {
                  return_value = SPI_CALLBACK_RECYCLE;   // Nominal outcome. Re-run the job.
                }
                break;

              case State::STAGE_3:
                sample_backlog_acc[sb_next_write++ % 32]((int16_t)regValue(LSM9DS1_A_DATA_X), (int16_t)regValue(LSM9DS1_A_DATA_Y), (int16_t)regValue(LSM9DS1_A_DATA_Z));
                if (0 == pending_samples) {
                  // If we have received the last expected sample, see how many more there are.
                  if (32 >= sb_next_write) {
                    sb_next_write    = 0;
                    // Solidify calibration.
                    if (0 == calibrate_from_data()) {
                      set_state(State::STAGE_4);
                      step_state();
                    }
                    else {
                      // Failed a pass through INIT-3.
                      readRegister((uint8_t) LSM9DS1_AG_FIFO_SRC);
                    }
                  }
                  else {
                    readRegister((uint8_t) LSM9DS1_AG_FIFO_SRC);
                  }
                }
                else {
                  return_value = SPI_CALLBACK_RECYCLE;   // Nominal outcome. Re-run the job.
                }
                break;

              default:
                break;
            }
          }
          break;

        /* We don't address these registers byte-wise. Empty case for documentation's sake. */
        case LSM9DS1_A_DATA_Y:  break;
        case LSM9DS1_A_DATA_Z:  break;

        case LSM9DS1_G_DATA_X:
          if (pending_samples > 0) {
            pending_samples--;
            switch (getState()) {
              case State::STAGE_5:
                collect_reading_gyr();
                if (0 == pending_samples) {
                  set_state(State::STAGE_4);
                  step_state();
                }
                else {
                  return_value = SPI_CALLBACK_RECYCLE;   // Nominal outcome. Re-run the job.
                }
                break;

              case State::STAGE_3:
                sample_backlog_gyr[sb_next_write++ % 32]((int16_t)regValue(LSM9DS1_G_DATA_X), (int16_t)regValue(LSM9DS1_G_DATA_Y), (int16_t)regValue(LSM9DS1_G_DATA_Z));
                if (0 == pending_samples) {
                  // If we have received the last expected sample, see how many more there are.
                  if (32 >= sb_next_write) {
                    sb_next_write    = 0;
                    // Solidify calibration.
                    if (0 == calibrate_from_data()) {
                      set_state(State::STAGE_4);
                      step_state();
                    }
                    else {
                      // Failed a pass through INIT-3.
                      readRegister((uint8_t) LSM9DS1_AG_FIFO_SRC);
                    }
                  }
                  else {
                    readRegister((uint8_t) LSM9DS1_AG_FIFO_SRC);
                  }
                }
                else {
                  return_value = SPI_CALLBACK_RECYCLE;   // Nominal outcome. Re-run the job.
                }
                break;

              default:
                break;
            }
          }
          break;

        /* We don't address these registers byte-wise. Empty case for documentation's sake. */
        case LSM9DS1_G_DATA_Y:   break;
        case LSM9DS1_G_DATA_Z:   break;

        // Since this data is only valid when the mag data is, we'll heed it in that block.
        case LSM9DS1_AG_DATA_TEMP:
          collect_reading_temperature();
          break;

        case LSM9DS1_AG_WHO_AM_I:
          if (0x68 == value) {
            if (!present()) {
              set_state(State::STAGE_1);
              if (step_state()) {
                //integrator->init();   // Call this to kick the integrator into noticing our state change.
              }
            }
            else {
              // Nominal condition. Maybe we bulk-read? Nice to have verification....
            }
          }
          else {
            // We lost the IMU, perhaps...
            set_state(State::STAGE_0);
            error_condition = IMU_ERROR_WRONG_IDENTITY;
          }
          break;

        case LSM9DS1_G_INT_GEN_SRC:     /* The gyroscope interrupt status register. */
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
        case LSM9DS1_AG_STATUS_REG:      /* Status of the gyr data registers on the sensor. */
          if (value & 0x08) {                 // We have fresh data to fetch.
            if (!fire_preformed_bus_op(&preformed_busop_read_gyr) ) {
              // Take corrective action.
              if (getVerbosity() > 1) local_log.concat("\tFailed to fast-read gyr vector\n");
            }
          }
          break;

        case LSM9DS1_AG_STATUS_REG_ALT:
          if (getVerbosity() > 5) local_log.concatf("\t LSM9DS1_AG_STATUS_REG_ALT: 0x%02x\n", (uint8_t) value);
          break;
        case LSM9DS1_AG_FIFO_CTRL:
          if (getVerbosity() > 5) local_log.concatf("\t XM_FIFO Control: 0x%02x\n", (uint8_t) value);
          break;
        case LSM9DS1_G_CTRL_REG1:
          if (getVerbosity() > 5) local_log.concatf("\t LSM9DS1_G_CTRL_REG1: 0x%02x\n", (uint8_t) value);
          if ((value >> 4) < MAXIMUM_RATE_INDEX_AG)  update_rate_acc = (value >> 4) & 0x0F;
          break;
        case LSM9DS1_AG_CTRL_REG4:
          if (getVerbosity() > 5) local_log.concatf("\t LSM9DS1_AG_CTRL_REG4: 0x%02x\n", (uint8_t) value);
          if (((value >> 3) & 0x07) < MAXIMUM_GAIN_INDEX_ACC)  scale_acc = (value >> 3) & 0x07;
          base_filter_param = (value >> 6) & 0x03;
          break;
        case LSM9DS1_A_CTRL_REG6:
          if (getVerbosity() > 5) local_log.concatf("\t LSM9DS1_A_CTRL_REG6: 0x%02x\n", (uint8_t) value);
          if (((value >> 3) & 0x03) < MAXIMUM_GAIN_INDEX_GYR)  scale_gyr = (value >> 3) & 0x03;
          break;
        case LSM9DS1_AG_FIFO_SRC:     /* The FIFO status register. */
          //if (getVerbosity() > 5) local_log.concatf("\t XM FIFO Status: 0x%02x\n", (uint8_t) value);
          if (initComplete()) {
            *pending_samples = value & 0x1F;
            if (!(value & 0x20)) {              // If the FIFO watermark is set and the FIFO is not empty...
              switch (getState()) {
                case State::STAGE_4:
                case State::STAGE_5:
                  if (State::STAGE_5 != desiredState()) {
                    break;
                  }
                  // Note: no break; on purpose.
                case State::STAGE_3:
                  if (preformed_busop_read_acc.isIdle()) {
                    if (!fire_preformed_bus_op(&preformed_busop_read_acc) ) {
                      if (getVerbosity() > 2) local_log.concat("\tFailed to fast-read accel vector\n");
                      error_condition = IMU_ERROR_BUS_INSERTION_FAILED;
                      if (getState() == State::STAGE_5) {
                        set_state(State::STAGE_4);
                      }
                    }
                    else if (State::STAGE_4 == getState()) {
                      set_state(State::STAGE_5);
                      if (step_state()) {
                        integrator->init();
                      }
                    }
                  }
                  break;

                default:
                  break;
              }
            }
            else if (getState() == State::STAGE_3) {
              readRegister((uint8_t) LSM9DS1_AG_FIFO_SRC);
            }
          }
          break;

        default:
          break;
      }
      if (op->devRegisterAdvance()) access_idx++;
    }

    if (profile()) {
      profiler_read_end = micros();
      if (getVerbosity() > 6) {
        local_log.concatf("\t Operation took %uus\n", (unsigned long) profiler_read_end);
      }
    }
  }


  /* WRITE Case-offs */
  else if (BusOpcode::TX == op->get_opcode()) {
    while ((access_len > 0) && regExists(access_idx)) {
      value = regValue(access_idx);
      access_len -= 1;   // Subtract the length.

      if (getVerbosity() > 3) {
        local_log.concatf("\t XM W: access_idx  0x%02x   (0x%04x)\n", access_idx, (uint16_t) value);
      }

      /* If we are doing a WRITE, most likey, we will only case to set some variable
           or parameter in the class now that we know the value is in the sensor.  */
      //reg_defs[access_idx].dirty = false;

      if (initPending()) {
        if (IDX_T1 == access_idx) {
        set_state(State::STAGE_2);
          if (step_state()) {
             //integrator->init();
          }
        }
      }

      switch (access_idx) {
        // TODO: We need to implement these....
        case LSM9DS1_AG_INT1_CTRL:
          break;
        case LSM9DS1_AG_INT2_CTRL:
          break;

        case LSM9DS1_G_CTRL_REG1:
          //if ((value >> 4) < MAXIMUM_RATE_INDEX_AG)  update_rate_acc = (value >> 4) & 0x0F;
          break;
        case LSM9DS1_G_CTRL_REG2:
          if (((value >> 3) & 0x07) < MAXIMUM_GAIN_INDEX_ACC)  scale_acc = (value >> 3) & 0x07;
          break;
        case LSM9DS1_A_CTRL_REG5:
          //if (((value >> 2) & 0x07) < MAXIMUM_RATE_INDEX_MAG)  update_rate_mag = ((value >> 2) & 0x07) + 1;
          break;
        case LSM9DS1_A_CTRL_REG6:
          if (((value >> 5) & 0x03) < MAXIMUM_GAIN_INDEX_GYR)  scale_gyr = (value >> 5) & 0x03;
          break;
        case LSM9DS1_AG_CTRL_REG8:
          if (value & 0x01) { // Did we write here to reset?
            if (!present()) {
              integrator->init();
            }
          }
          break;


        default:
          if (getVerbosity() > 5) local_log.concatf("\t XM Wrote an unimplemented register.\n");
          break;
      }
      if (op->devRegisterAdvance()) access_idx++;
    }
  }

  if (local_log.length() > 0) Kernel::log(&local_log);
  return return_value;
}
