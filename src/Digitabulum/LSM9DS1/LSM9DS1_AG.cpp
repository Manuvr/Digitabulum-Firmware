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

#include "LSM9DS1_AG.h"
#include "IIU.h"
#include "../ManuLegend/ManuLegend.h"


/****************************************************************************************************
*      _______.___________.    ___   .___________. __    ______     _______.
*     /       |           |   /   \  |           ||  |  /      |   /       |
*    |   (----`---|  |----`  /  ^  \ `---|  |----`|  | |  ,----'  |   (----`
*     \   \       |  |      /  /_\  \    |  |     |  | |  |        \   \
* .----)   |      |  |     /  _____  \   |  |     |  | |  `----.----)   |
* |_______/       |__|    /__/     \__\  |__|     |__|  \______|_______/
*
* Static members and initializers should be located here. Initializers first, functions second.
****************************************************************************************************/
/*
* This is a table of frequencies versus period (in micros).
*/
const UpdateRate2Hertz LSM9DS1_AG::rate_settings_acc[MAXIMUM_RATE_INDEX_AG] = {
  {0.0,  0.0f},
  {14.9, (1/14.9f)},
  {59.5, (1/59.5f)},
  {119,  (1/119.0f)},
  {238,  (1/238.0f)},
  {476,  (1/476.0f)},
  {952,  (1/952.0f)}
};

const UpdateRate2Hertz LSM9DS1_AG::rate_settings_gyr[MAXIMUM_RATE_INDEX_AG] = {
  {0.0,  0.0f},
  {10.0, (1/10.0f)},
  {50.0, (1/50.0f)},
  {119,  (1/119.0f)},
  {238,  (1/238.0f)},
  {476,  (1/476.0f)},
  {952,  (1/952.0f)}
};


/*
* This is a generic table of scales versus unit-per-bit for 16-bit types.
* This needs to be updated with actual datasheet values.
*/
const GainErrorMap LSM9DS1_AG::error_map_acc[MAXIMUM_GAIN_INDEX_ACC] = {
  {2,  (2/32768.0f),  0.000030},
  {4,  (4/32768.0f),  0.000061},
  {6,  (6/32768.0f),  0.000092},
  {8,  (8/32768.0f),  0.000122},
  {16, (16/32768.0f), 0.000244}
};

const GainErrorMap LSM9DS1_AG::error_map_gyr[MAXIMUM_GAIN_INDEX_GYR] = {
  {245,   (245/32768.0f),  0.00437 * 0.0174532777778},
  {500,   (500/32768.0f),  0.00875 * 0.0174532777778},
  {2000,  (2000/32768.0f), 0.03500 * 0.0174532777778}
};

const float LSM9DS1_AG::max_range_vect_acc  = 16.0;
const float LSM9DS1_AG::max_range_vect_gyr  = 2000.0;


/****************************************************************************************************
*                       _                               _
*     /\               | |                             | |
*    /  \   ___ ___ ___| | ___ _ __ ___  _ __ ___   ___| |_ ___ _ __
*   / /\ \ / __/ __/ _ \ |/ _ \ '__/ _ \| '_ ` _ \ / _ \ __/ _ \ '__|
*  / ____ \ (_| (_|  __/ |  __/ | | (_) | | | | | |  __/ ||  __/ |
* /_/    \_\___\___\___|_|\___|_|  \___/|_| |_| |_|\___|\__\___|_|
*
****************************************************************************************************/
/*
* Accelerometer data
* Tests all the required registers for freshness and builds a Vector with the new reading
*   if possible.
* Returns...
*   0 on success with no vector.
*   1 on success with new vector.
*  -1 on error.
*/
int8_t LSM9DS1_AG::collect_reading_acc() {
  reg_defs[LSM9DS1_A_DATA_X].unread = false;
  reg_defs[LSM9DS1_A_DATA_Y].unread = false;
  reg_defs[LSM9DS1_A_DATA_Z].unread = false;

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

  Vector3<int16_t> reflection_vector_acc(LegendManager::reflection_acc.x, LegendManager::reflection_acc.y, LegendManager::reflection_acc.z);

  if (cancel_error) {
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
int8_t LSM9DS1_AG::request_rescale_acc(uint8_t nu_scale_idx) {
  if (nu_scale_idx < MAXIMUM_GAIN_INDEX_ACC) {
    if (scale_acc != nu_scale_idx) {
      uint8_t temp8 = regValue(LSM9DS1_A_CTRL_REG6);
      temp8 =  (temp8 & 0xE7) | (nu_scale_idx << 3);
      return SPIDeviceWithRegisters::writeRegister(LSM9DS1_A_CTRL_REG6, temp8);
    }
    return -2;
  }
  return -1;
}


/**
* Call to alter sample rate.
*/
int8_t LSM9DS1_AG::set_sample_rate_acc(uint8_t nu_srate_idx) {
  if (nu_srate_idx < MAXIMUM_RATE_INDEX_AG) {
    if (update_rate_acc != nu_srate_idx) {
      uint8_t temp8 = regValue(LSM9DS1_A_CTRL_REG6);
      temp8 =  (temp8 & 0x1F) | (nu_srate_idx << 5);
      update_rate_acc = nu_srate_idx;
      return SPIDeviceWithRegisters::writeRegister(LSM9DS1_A_CTRL_REG6, temp8);
    }
    return -2;
  }
  return -1;
}


// Call to change the bandwidth of the AA filter.
int8_t LSM9DS1_AG::set_base_filter_param_acc(uint8_t nu_bw_idx) {
  if ((nu_bw_idx < 4) && (nu_bw_idx != base_filter_param)) {
    base_filter_param = nu_bw_idx;
    uint8_t temp8 = regValue(LSM9DS1_A_CTRL_REG6);
    temp8 =  (temp8 & 0xFC) | (nu_bw_idx & 0x03);
    return SPIDeviceWithRegisters::writeRegister(LSM9DS1_A_CTRL_REG6, temp8);
  }
  return -1;
}






/****************************************************************************************************
*   _____
*  / ____|
* | |  __ _   _ _ __ ___
* | | |_ | | | | '__/ _ \
* | |__| | |_| | | | (_) |
*  \_____|\__, |_|  \___/
*          __/ |
*         |___/
****************************************************************************************************/

/*
* Gyroscope data
* Tests all the required registers for freshness and builds a Vector with the new reading
*   if possible.
* Returns...
*   0 on success with no vector.
*   1 on success with new vector.
*  -1 on error.
*/
int8_t LSM9DS1_AG::collect_reading_gyr() {
  reg_defs[LSM9DS1_G_DATA_X].unread = false;
  reg_defs[LSM9DS1_G_DATA_Y].unread = false;
  reg_defs[LSM9DS1_G_DATA_Z].unread = false;

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

  Vector3<int16_t> reflection_vector_gyr(LegendManager::reflection_gyr.x, LegendManager::reflection_gyr.y, LegendManager::reflection_gyr.z);

  if (cancel_error) {
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
int8_t LSM9DS1_AG::request_rescale_gyr(uint8_t nu_scale_idx) {
  if (nu_scale_idx < MAXIMUM_GAIN_INDEX_ACC) {
    if (scale_acc != nu_scale_idx) {
      if (verbosity > 2) Kernel::log("request_rescale_gyr():\tRescaling Gyro.\n");
      uint8_t temp8 = regValue(LSM9DS1_G_CTRL_REG1);
      temp8 =  (temp8 & 0xE7) | (nu_scale_idx << 3);
      return SPIDeviceWithRegisters::writeRegister(LSM9DS1_G_CTRL_REG1, temp8);
    }
    return -2;
  }
  return -1;
}


/**
* Call to alter sample rate.
*/
int8_t LSM9DS1_AG::set_sample_rate_gyr(uint8_t nu_srate_idx) {
  if (nu_srate_idx < MAXIMUM_RATE_INDEX_AG) {
    if (update_rate_acc != nu_srate_idx) {
      if (verbosity > 2) Kernel::log("set_sample_rate_gyr():\t\n");
      uint8_t temp8 = regValue(LSM9DS1_G_CTRL_REG1);
      temp8 =  (temp8 & 0x1F) | (nu_srate_idx << 5);
      update_rate_gyr = nu_srate_idx;
      return SPIDeviceWithRegisters::writeRegister(LSM9DS1_G_CTRL_REG1, temp8);
    }
    return -2;
  }
  return -1;
}



// Call to change the cutoff of the gyro's base filter.
int8_t LSM9DS1_AG::set_base_filter_param_gyr(uint8_t nu_bw_idx) {
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
int8_t LSM9DS1_AG::collect_reading_temperature() {
  int8_t return_value = 1;
  reg_defs[LSM9DS1_AG_DATA_TEMP].unread = false;
  integrator->setTemperature(((int16_t)regValue(LSM9DS1_AG_DATA_TEMP)) / 1.0f);
  //integrator->setTemperature(((int16_t) value) / 4.0f);
  return return_value;
}





/****************************************************************************************************
*   ___ _              ___      _ _              _      _
*  / __| |__ _ ______ | _ ) ___(_) |___ _ _ _ __| |__ _| |_ ___
* | (__| / _` (_-<_-< | _ \/ _ \ | / -_) '_| '_ \ / _` |  _/ -_)
*  \___|_\__,_/__/__/ |___/\___/_|_\___|_| | .__/_\__,_|\__\___|
*                                          |_|
* Constructors/destructors, class initialization functions and so-forth...
****************************************************************************************************/

/*
( This accelerometer has 49 registers (as we choose to carve them).
*/
LSM9DS1_AG::LSM9DS1_AG(uint8_t address, IIU* _integrator) : LSM9DSx_Common("XM ", address, _integrator, 49) {
  // First, we should define our registers....
  // 40 registers. 23 bytes
  register_pool = (uint8_t*) malloc(49);

  // Now we should give them initial definitions. This is our chance to set default configs.
  // Any config which we want written during init() should have dirty set to true.
  reg_defs[LSM9DS1_AG_ACT_THS]         = DeviceRegister((bus_addr + 0x04), (uint8_t)  0x00, (register_pool +  0), false, false, true);
  reg_defs[LSM9DS1_AG_ACT_DUR]         = DeviceRegister((bus_addr + 0x05), (uint8_t)  0x00, (register_pool +  1), false, false, true);
  reg_defs[LSM9DS1_A_INT_GEN_CFG]      = DeviceRegister((bus_addr + 0x06), (uint8_t)  0x00, (register_pool +  2), false, false, true);
  reg_defs[LSM9DS1_A_INT_GEN_THS_X]    = DeviceRegister((bus_addr + 0x07), (uint8_t)  0x00, (register_pool +  3), false, false, true);
  reg_defs[LSM9DS1_A_INT_GEN_THS_Y]    = DeviceRegister((bus_addr + 0x08), (uint8_t)  0x00, (register_pool +  4), false, false, true);
  reg_defs[LSM9DS1_A_INT_GEN_THS_Z]    = DeviceRegister((bus_addr + 0x09), (uint8_t)  0x00, (register_pool +  5), false, false, true);
  reg_defs[LSM9DS1_A_INT_GEN_DURATION] = DeviceRegister((bus_addr + 0x0A), (uint8_t)  0x00, (register_pool +  6), false, false, true);
  reg_defs[LSM9DS1_G_REFERENCE]        = DeviceRegister((bus_addr + 0x0B), (uint8_t)  0x00, (register_pool +  7), false, false, true);
  reg_defs[LSM9DS1_AG_INT1_CTRL]       = DeviceRegister((bus_addr + 0x0C), (uint8_t)  0x00, (register_pool +  8), false, false, true);
  reg_defs[LSM9DS1_AG_INT2_CTRL]       = DeviceRegister((bus_addr + 0x0D), (uint8_t)  0x00, (register_pool +  9), false, false, true);
  reg_defs[LSM9DS1_AG_WHO_AM_I]        = DeviceRegister((bus_addr + 0x0F), (uint8_t)  0x00, (register_pool + 10), false, false, true);
  reg_defs[LSM9DS1_G_CTRL_REG1]        = DeviceRegister((bus_addr + 0x10), (uint8_t)  0x00, (register_pool + 11), false, false, true);
  reg_defs[LSM9DS1_G_CTRL_REG2]        = DeviceRegister((bus_addr + 0x11), (uint8_t)  0x00, (register_pool + 12), false, false, true);
  reg_defs[LSM9DS1_G_CTRL_REG3]        = DeviceRegister((bus_addr + 0x12), (uint8_t)  0x00, (register_pool + 13), false, false, true);
  reg_defs[LSM9DS1_G_ORIENT_CFG]       = DeviceRegister((bus_addr + 0x13), (uint8_t)  0x00, (register_pool + 14), false, false, true);
  reg_defs[LSM9DS1_G_INT_GEN_SRC]      = DeviceRegister((bus_addr + 0x14), (uint8_t)  0x00, (register_pool + 15), false, false, true);
  reg_defs[LSM9DS1_AG_DATA_TEMP]       = DeviceRegister((bus_addr + 0x15), (uint16_t) 0x00, (register_pool + 16), false, false, true);
  reg_defs[LSM9DS1_AG_STATUS_REG]      = DeviceRegister((bus_addr + 0x17), (uint8_t)  0x00, (register_pool + 18), false, false, true);
  reg_defs[LSM9DS1_G_DATA_X]           = DeviceRegister((bus_addr + 0x18), (uint16_t) 0x00, (register_pool + 19), false, false, true);
  reg_defs[LSM9DS1_G_DATA_Y]           = DeviceRegister((bus_addr + 0x1A), (uint16_t) 0x00, (register_pool + 21), false, false, true);
  reg_defs[LSM9DS1_G_DATA_Z]           = DeviceRegister((bus_addr + 0x1C), (uint16_t) 0x00, (register_pool + 23), false, false, true);
  reg_defs[LSM9DS1_AG_CTRL_REG4]       = DeviceRegister((bus_addr + 0x1E), (uint8_t)  0x38, (register_pool + 25), false, false, true);
  reg_defs[LSM9DS1_A_CTRL_REG5]        = DeviceRegister((bus_addr + 0x1F), (uint8_t)  0x38, (register_pool + 26), false, false, true);
  reg_defs[LSM9DS1_A_CTRL_REG6]        = DeviceRegister((bus_addr + 0x20), (uint8_t)  0x00, (register_pool + 27), false, false, true);
  reg_defs[LSM9DS1_A_CTRL_REG7]        = DeviceRegister((bus_addr + 0x21), (uint8_t)  0x00, (register_pool + 28), false, false, true);
  reg_defs[LSM9DS1_AG_CTRL_REG8]       = DeviceRegister((bus_addr + 0x22), (uint8_t)  0x04, (register_pool + 29), false, false, true);
  reg_defs[LSM9DS1_AG_CTRL_REG9]       = DeviceRegister((bus_addr + 0x23), (uint8_t)  0x00, (register_pool + 30), false, false, true);
  reg_defs[LSM9DS1_AG_CTRL_REG10]      = DeviceRegister((bus_addr + 0x24), (uint8_t)  0x00, (register_pool + 31), false, false, true);
  reg_defs[LSM9DS1_A_INT_GEN_SRC]      = DeviceRegister((bus_addr + 0x26), (uint8_t)  0x00, (register_pool + 32), false, false, true);
  reg_defs[LSM9DS1_AG_STATUS_REG_ALT]  = DeviceRegister((bus_addr + 0x27), (uint8_t)  0x00, (register_pool + 33), false, false, true);
  reg_defs[LSM9DS1_A_DATA_X]           = DeviceRegister((bus_addr + 0x28), (uint16_t) 0x00, (register_pool + 34), false, false, true);
  reg_defs[LSM9DS1_A_DATA_Y]           = DeviceRegister((bus_addr + 0x2A), (uint16_t) 0x00, (register_pool + 36), false, false, true);
  reg_defs[LSM9DS1_A_DATA_Z]           = DeviceRegister((bus_addr + 0x2C), (uint16_t) 0x00, (register_pool + 38), false, false, true);
  reg_defs[LSM9DS1_AG_FIFO_CTRL]       = DeviceRegister((bus_addr + 0x2E), (uint8_t)  0x00, (register_pool + 40), false, false, true);
  reg_defs[LSM9DS1_AG_FIFO_SRC]        = DeviceRegister((bus_addr + 0x2F), (uint8_t)  0x00, (register_pool + 41), false, false, true);
  reg_defs[LSM9DS1_G_INT_GEN_CFG]      = DeviceRegister((bus_addr + 0x30), (uint8_t)  0x00, (register_pool + 42), false, false, true);
  reg_defs[LSM9DS1_G_INT_GEN_THS_X]    = DeviceRegister((bus_addr + 0x31), (uint16_t) 0x00, (register_pool + 43), false, false, true);
  reg_defs[LSM9DS1_G_INT_GEN_THS_Y]    = DeviceRegister((bus_addr + 0x33), (uint16_t) 0x00, (register_pool + 45), false, false, true);
  reg_defs[LSM9DS1_G_INT_GEN_THS_Z]    = DeviceRegister((bus_addr + 0x35), (uint16_t) 0x00, (register_pool + 47), false, false, true);
  reg_defs[LSM9DS1_G_INT_GEN_DURATION] = DeviceRegister((bus_addr + 0x37), (uint8_t)  0x00, (register_pool + 49), false, false, true);


  /* Certain register indicies serve a common purpose in the base class. Define those... */
  idx_identity     = LSM9DS1_AG_WHO_AM_I;
  idx_io_test_0    = LSM9DS1_G_INT_GEN_THS_X;
  idx_io_test_1    = LSM9DS1_G_INT_GEN_THS_Z;

  // Preform our most commonly-used bus operations to minimize thrash and other kinds of overhead.
  preformed_busop_read_acc.shouldReap(false);
  preformed_busop_read_acc.devRegisterAdvance(true);
  preformed_busop_read_acc.callback = (SPIDeviceWithRegisters*) this;
  preformed_busop_read_acc.opcode   = BusOpcode::RX;
  preformed_busop_read_acc.reg_idx  = LSM9DS1_A_DATA_X;
  preformed_busop_read_acc.buf      = reg_defs[preformed_busop_read_acc.reg_idx].val;
  preformed_busop_read_acc.bus_addr = reg_defs[preformed_busop_read_acc.reg_idx].addr | 0x80;
  preformed_busop_read_acc.buf_len  = 6;

  preformed_busop_read_gyr.shouldReap(false);
  preformed_busop_read_gyr.devRegisterAdvance(true);
  preformed_busop_read_gyr.callback = (SPIDeviceWithRegisters*) this;
  preformed_busop_read_gyr.opcode   = BusOpcode::RX;
  preformed_busop_read_gyr.reg_idx  = LSM9DS1_G_DATA_X;
  preformed_busop_read_gyr.buf      = reg_defs[preformed_busop_read_gyr.reg_idx].val;
  preformed_busop_read_gyr.bus_addr = reg_defs[preformed_busop_read_gyr.reg_idx].addr | 0x80;
  preformed_busop_read_gyr.buf_len  = 6;

  preformed_busop_irq_0.shouldReap(false);
  preformed_busop_irq_0.devRegisterAdvance(true);
  preformed_busop_irq_0.callback = (SPIDeviceWithRegisters*) this;
  preformed_busop_irq_0.opcode   = BusOpcode::RX;
  preformed_busop_irq_0.reg_idx  = LSM9DS1_G_INT_GEN_SRC;
  preformed_busop_irq_0.bus_addr = reg_defs[preformed_busop_irq_0.reg_idx].addr | 0x80;
  preformed_busop_irq_0.buf      = reg_defs[preformed_busop_irq_0.reg_idx].val;
  preformed_busop_irq_0.buf_len  = 22;

  preformed_busop_irq_1.shouldReap(false);
  preformed_busop_irq_1.devRegisterAdvance(true);
  preformed_busop_irq_1.callback = (SPIDeviceWithRegisters*) this;
  preformed_busop_irq_1.opcode   = BusOpcode::RX;
  preformed_busop_irq_1.reg_idx  = LSM9DS1_A_INT_GEN_SRC;
  preformed_busop_irq_1.bus_addr = reg_defs[preformed_busop_irq_1.reg_idx].addr | 0x80;
  preformed_busop_irq_1.buf      = reg_defs[preformed_busop_irq_1.reg_idx].val;
  preformed_busop_irq_1.buf_len  = 3;

  full_register_refresh.shouldReap(false);
  full_register_refresh.devRegisterAdvance(true);
  full_register_refresh.callback = (SPIDeviceWithRegisters*) this;
  full_register_refresh.opcode   = BusOpcode::RX;
  full_register_refresh.reg_idx  = LSM9DS1_AG_FIFO_CTRL;
  full_register_refresh.bus_addr = reg_defs[full_register_refresh.reg_idx].addr | 0x80;
  full_register_refresh.buf      = reg_defs[full_register_refresh.reg_idx].val;
  full_register_refresh.buf_len  = 18;

  // Local class stuff...
  last_val_acc(0.0f, 0.0f, 0.0f);
  autoscale_acc       = false;
  scale_acc           = 0;
  update_rate_acc     = 0;
  discards_remain_acc = 0;
  discards_total_acc  = 0;

  last_val_gyr(0.0f, 0.0f, 0.0f);
  autoscale_gyr       = false;
  scale_gyr           = 0;
  update_rate_gyr     = 0;
  discards_remain_gyr = 0;
  discards_total_gyr  = 0;

  // Superclass stuff...
  init();
}


LSM9DS1_AG::~LSM9DS1_AG(void) {
  // This class will never be torn down.
}



/****************************************************************************************************
* Members to be called from the integrator.                                                         *
****************************************************************************************************/
int8_t LSM9DS1_AG::readSensor(void) {
  int8_t return_value = 0;
  if (!present()) {
    return -1;
  }
  if (!calibrated()) {
    //return IMU_ERROR_NOT_CALIBRATED;
  }

  if (initComplete()) {
    // If there is more data on the way, we will let the callback do this for us.
    if (preformed_busop_read_acc.isIdle()) {
      readRegister((uint8_t) LSM9DS1_AG_FIFO_SRC);
    }
  }

  if (preformed_busop_irq_1.isIdle()) {
    fire_preformed_bus_op(&preformed_busop_irq_1);
  }
  return return_value;
}



/*
* Reads every register in the sensor with maximum bus efficiency.
*/
int8_t LSM9DS1_AG::bulk_refresh() {
  if (verbosity > 3) Kernel::log("XM::bulk_refresh()\n");
  if (!present()) {
    return readRegister((uint8_t) LSM9DS1_AG_WHO_AM_I);
  }

  if (fire_preformed_bus_op(&preformed_busop_irq_0)) {
    if (fire_preformed_bus_op(&preformed_busop_irq_1)) {
      return LSM9DSx_Common::bulk_refresh();
    }
    else {
      if (verbosity > 2) Kernel::log("\t Failed to fire preform irq_xm1\n");
    }
  }
  else {
    if (verbosity > 2) Kernel::log("\t Failed to fire preform irq_xm0\n");
  }

  return -1;
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
* Stage 1) Process register data and service interrupt if applicable in spi_op_callback().
*
* Return codes:
*   Success, with IRQ service
*   Success, no IRQ service
*   Failure
*/
int8_t LSM9DS1_AG::irq_0() {
  int8_t return_value = IMU_ERROR_NO_ERROR;

  if (verbosity > 3) Kernel::log("XM::irq_0()\n");
  if (initComplete()) {
    if (!fire_preformed_bus_op(&preformed_busop_irq_0) ) {
      // Take corrective action.
    }
  }
  return return_value;
}

int8_t LSM9DS1_AG::irq_1() {
  int8_t return_value = IMU_ERROR_NO_ERROR;

  if (verbosity > 3) Kernel::log("XM::irq_1()\n");
  if (initComplete()) {
    //readRegister((uint8_t) LSM9DS1_AG_STATUS_REG_M);
    //readRegister((uint8_t) LSM9DS1_AG_FIFO_SRC_REG);
    if (!fire_preformed_bus_op(&preformed_busop_irq_1) ) {
      // Take corrective action.
    }
  }
  return return_value;
}



void LSM9DS1_AG::reset() {
  autoscale_acc       = false;
  scale_acc           = 0;
  update_rate_acc     = 0;
  discards_remain_acc = 0;
  discards_total_acc  = 0;

  autoscale_gyr       = false;
  scale_gyr           = 0;
  update_rate_gyr     = 0;
  discards_remain_gyr = 0;
  discards_total_gyr  = 0;

  preformed_busop_irq_0.xfer_state = XferState::IDLE;
  preformed_busop_irq_1.xfer_state = XferState::IDLE;

  noise_floor_acc.set(0.0f, 0.0f, 0.0f);
  noise_floor_gyr.set(0.0f, 0.0f, 0.0f);

  LSM9DSx_Common::reset();
  SPIDeviceWithRegisters::writeRegister(LSM9DS1_AG_CTRL_REG8, 0x01);
}



// Init parameters for the inertial aspect. Starting at CTRL_REG1_G.
uint8_t bulk_init_block_ag_0[4] = {
  0b10001000,  // 238Hz ODR, 14Hz cutoff, 500dps
  0b00000000,  // Low-pass filer feeds FIFO and interrupt logic directly.
  0b00000000,  // No high-pass parameters.
  0b00000000   // Orientation default values.
};

// Init parameters for the inertial aspect. Starting at CTRL_REG5_XL.
uint8_t bulk_init_block_ag_1[6] = {
  0b00111000,  // No decimation, all axes enabled.
  0b10010000,  // 238Hz ODR, 4g range, max AA bandwidth.
  0b10000101,  // High-res, ODR/50 LP-cutoff, filter feeds FIFO and interrupt logic.
  0b00000100,  // Non-blocked data reg update, IRQ pins push-pull and active-high,
  0b00000110,  // No sleepy gyro, FIFO enabled, i2c disabled.
  0b01100000   // No self-tests.
};

// Init parameters for the inertial aspect. Starting at LSM9DS1_G_INT_GEN_CFG.
uint8_t bulk_init_block_ag_2[8] = {
  0b01111111,  // Gyro OR IRQ conditions, latched, all axes.
  0b00000000,  // Gyro IRQ thresholds.
  0b10000000,  // Gyro IRQ thresholds.
  0b00000000,  // Gyro IRQ thresholds.
  0b10000000,  // Gyro IRQ thresholds.
  0b00000000,  // Gyro IRQ thresholds.
  0b10000000,  // Gyro IRQ thresholds.
  0b10010100   // 20 samples above threshold required to trigger IRQ.
};



uint8_t sample_rate_block_x[] = { 0b00000000, 0b00000000 };
uint8_t sample_rate_block_m[] = { 0b00000000, 0b00000000 };

/****************************************************************************************************
* Runtime settings and status members.                                                              *
****************************************************************************************************/

int8_t LSM9DS1_AG::configure_sensor() {
  if (verbosity > 3) Kernel::log("XM::configure_sensor()\n");

  writeRegister(LSM9DS1_M_CTRL_REG1,   (uint8_t*) &bulk_init_block_ag_0, 4);
  writeRegister(LSM9DS1_M_CTRL_REG5,   (uint8_t*) &bulk_init_block_ag_1, 6);
  writeRegister(LSM9DS1_G_INT_GEN_CFG, (uint8_t*) &bulk_init_block_ag_2, 8);

  // Continuous FIFO. Half-full (16) is threshold.
  SPIDeviceWithRegisters::writeRegister(LSM9DS1_AG_FIFO_CTRL, (uint8_t)  0b11010000);

  // If we are "init-pending", but not in readback phase, we need to dispatch the test writes.
  write_test_bytes();
  //writeDirtyRegisters();
  return 0;
}


int8_t LSM9DS1_AG::calibrate_from_data() {
  // Is reading stable?

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
  Vector3<int16_t> reflection_vector_gyr(LegendManager::reflection_gyr.x, LegendManager::reflection_gyr.y, LegendManager::reflection_gyr.z);

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


/*
*
*/
bool LSM9DS1_AG::is_setup_completed() {
  if (!present()) return false;

  if (!initComplete()) {   // TODO: Redundant.
    if (initPending()) {   // TODO: Redundant.
      if (reg_defs[LSM9DS1_M_CTRL_REG1].dirty) return false;
      if (reg_defs[LSM9DS1_M_CTRL_REG5].dirty) return false;
      if (reg_defs[LSM9DS1_G_INT_GEN_CFG].dirty) return false;
      if (reg_defs[LSM9DS1_AG_FIFO_CTRL].dirty) return false;
    }
  }

  return true;
}





/****************************************************************************************************
* Debugging and logging aides...                                                                    *
****************************************************************************************************/
/*
* Dump the contents of this device to the logger.
* This is an override.
*/
void LSM9DS1_AG::dumpDevRegs(StringBuilder *output) {
  if (NULL == output) return;
  LSM9DSx_Common::dumpDevRegs(output);

  if (verbosity > 1) {
    output->concatf("--- update_rate_acc     %3.0f Hz\n", (double) rate_settings_acc[update_rate_acc].hertz);
  }
  if (verbosity > 2) {
    output->concatf("--- scale_acc           +/-%d\n", error_map_acc[scale_acc].scale);
    output->concatf("--- autoscale_acc       %s\n", (autoscale_acc ? "yes" : "no"));
    output->concatf("--- noise_floor_acc     (%d, %d, %d)\n", noise_floor_acc.x, noise_floor_acc.y, noise_floor_acc.z);
  }

  if (verbosity > 1) {
    output->concatf("--- update_rate_gyr     %3.0f Hz\n", (double) rate_settings_gyr[update_rate_gyr].hertz);
  }
  if (verbosity > 2) {
    output->concatf("--- scale_gyr           +/-%d\n", error_map_gyr[scale_gyr].scale);
    output->concatf("--- autoscale_gyr       %s\n\n", (autoscale_gyr ? "yes" : "no"));
    output->concatf("--- noise_floor_gyr     (%d, %d, %d)\n", noise_floor_gyr.x, noise_floor_gyr.y, noise_floor_gyr.z);
  }

  if (verbosity > 3) SPIDeviceWithRegisters::dumpDevRegs(output);
}


/*
* Dump the contents of this device to the logger.
* This is an override.
*/
void LSM9DS1_AG::dumpPreformedElements(StringBuilder *output) {
  output->concat("--- Accel preformed elements\n");
  LSM9DSx_Common::dumpPreformedElements(output);


  output->concat("--- Vector read (Accel)\n");
  preformed_busop_read_acc.printDebug(output);

  output->concat("--- Vector read (Mag)\n");
  preformed_busop_read_gyr.printDebug(output);

  output->concat("--- IRQ pin1\n");
  preformed_busop_irq_0.printDebug(output);
  output->concat("--- IRQ pin2\n");
  preformed_busop_irq_1.printDebug(output);
  output->concat("\n");
}



/****************************************************************************************************
* Overrides from the SPI apparatus...                                                               *
****************************************************************************************************/

/*
* All notifications of bus activity enter the class here. This is probably where
*   we should act on data coming in.
*/
int8_t LSM9DS1_AG::spi_op_callback(SPIBusOp* op) {
  int8_t return_value = SPI_CALLBACK_NOMINAL;

  if (-1 == op->reg_idx) {
    // Our class implementation considers this a serious problem.
    if (verbosity > 3) Kernel::log("~~~~~~~~LSM9DS1_AG::spi_op_callback   (ERROR CASE -2)\n");
    error_condition = (BusOpcode::RX == op->opcode) ? IMU_ERROR_BUS_OPERATION_FAILED_R : IMU_ERROR_BUS_OPERATION_FAILED_W;
    return SPI_CALLBACK_ERROR;
  }

  // There is zero chance this object will be a null pointer unless it was done on purpose.
  if (op->hasError()) {
    if (verbosity > 3) {
      local_log.concat("~~~~~~~~LSM9DS1_AG::spi_op_callback   (ERROR CASE -1)\n");
      op->printDebug(&local_log);
      Kernel::log(&local_log);
    }
    error_condition = (BusOpcode::RX == op->opcode) ? IMU_ERROR_BUS_OPERATION_FAILED_R : IMU_ERROR_BUS_OPERATION_FAILED_W;

    // TODO: Should think carefully, and...   return_value = SPI_CALLBACK_RECYCLE;   // Re-run the job.
    return SPI_CALLBACK_ERROR;
  }

  uint8_t access_len = op->buf_len;  // The access length lets us know how many things changed.
  int16_t access_idx = op->reg_idx;  // The access length lets us know how many things changed.
  unsigned int value = regValue(access_idx);
  if (verbosity > 6) local_log.concatf("%s  XM::spi_op_callback(0x%08x): value: %d \t access_idx  %d \t access_len: %d\n", op->getOpcodeString(), (uint32_t)((SPIOpCallback*) this), value, access_idx, access_len);

  /* Our first choice is: Did we just finish a WRITE or a READ? */
  /* READ Case-offs */
  if (BusOpcode::RX == op->opcode) {
    while ((access_len > 0) && (access_idx < reg_count)) {
      value = regValue(access_idx);
      access_len -= reg_defs[access_idx].len;   // Subtract the length.
      reg_defs[access_idx].unread = false;

      if (verbosity > 3) {
        local_log.concatf("\t XM R: access_idx  0x%02x   (0x%04x)\n", access_idx, (uint16_t) value);
      }

      if (initPending()) {
        if (idx_io_test_1 == access_idx) {
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
              if (autoscale_gyr) request_rescale_gyr(scale_gyr+1);
            }
            else if (value & 0x1C) {          // Did we drop below our set threshold?
              if (autoscale_gyr) request_rescale_gyr(scale_gyr-1);
            }
          }
          else if (value & 0x02) {            // We had a range overflow. Means we need to autoscale...
            if (autoscale_gyr) request_rescale_gyr(scale_gyr+1);
          }
          break;
        // TODO: We need to implement these....
        case LSM9DS1_AG_STATUS_REG:      /* Status of the gyr data registers on the sensor. */
          if (value & 0x08) {                 // We have fresh data to fetch.
            if (!fire_preformed_bus_op(&preformed_busop_read_gyr) ) {
              // Take corrective action.
              if (verbosity > 1) local_log.concat("\tFailed to fast-read gyr vector\n");
            }
          }
          break;

        case LSM9DS1_AG_STATUS_REG_ALT:
          if (verbosity > 5) local_log.concatf("\t LSM9DS1_AG_STATUS_REG_ALT: 0x%02x\n", (uint8_t) value);
          break;
        case LSM9DS1_AG_FIFO_CTRL:
          if (verbosity > 5) local_log.concatf("\t XM_FIFO Control: 0x%02x\n", (uint8_t) value);
          break;
        case LSM9DS1_G_CTRL_REG1:
          if (verbosity > 5) local_log.concatf("\t LSM9DS1_G_CTRL_REG1: 0x%02x\n", (uint8_t) value);
          if ((value >> 4) < MAXIMUM_RATE_INDEX_AG)  update_rate_acc = (value >> 4) & 0x0F;
          break;
        case LSM9DS1_AG_CTRL_REG4:
          if (verbosity > 5) local_log.concatf("\t LSM9DS1_AG_CTRL_REG4: 0x%02x\n", (uint8_t) value);
          if (((value >> 3) & 0x07) < MAXIMUM_GAIN_INDEX_ACC)  scale_acc = (value >> 3) & 0x07;
          base_filter_param = (value >> 6) & 0x03;
          break;
        case LSM9DS1_A_CTRL_REG6:
          if (verbosity > 5) local_log.concatf("\t LSM9DS1_A_CTRL_REG6: 0x%02x\n", (uint8_t) value);
          if (((value >> 3) & 0x03) < MAXIMUM_GAIN_INDEX_GYR)  scale_gyr = (value >> 3) & 0x03;
          break;
        case LSM9DS1_AG_FIFO_SRC:     /* The FIFO status register. */
          //if (verbosity > 5) local_log.concatf("\t XM FIFO Status: 0x%02x\n", (uint8_t) value);
          if (initComplete()) {
            pending_samples = value & 0x1F;
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
                      if (verbosity > 2) local_log.concat("\tFailed to fast-read accel vector\n");
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

    if (profile) {
      profiler_read_end = micros();
      if (verbosity > 6) {
        local_log.concatf("\t Operation took %uus\n", (unsigned long) profiler_read_end);
      }
    }
  }


  /* WRITE Case-offs */
  else if (BusOpcode::TX == op->opcode) {
    while ((access_len > 0) && (access_idx < reg_count)) {
      value = regValue(access_idx);
      access_len -= reg_defs[access_idx].len;   // Subtract the length.

      if (verbosity > 3) {
        local_log.concatf("\t XM W: access_idx  0x%02x   (0x%04x)\n", access_idx, (uint16_t) value);
      }

      /* If we are doing a WRITE, most likey, we will only case to set some variable
           or parameter in the class now that we know the value is in the sensor.  */
      reg_defs[access_idx].dirty = false;

      if (initPending()) {
        if (idx_io_test_1 == access_idx) {
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
          if (verbosity > 5) local_log.concatf("\t XM Wrote an unimplemented register.\n");
          break;
      }
      if (op->devRegisterAdvance()) access_idx++;
    }
  }

  if (local_log.length() > 0) Kernel::log(&local_log);
  return return_value;
}
