/*
File:   LSM9DS1.cpp
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
#include "../CPLDDriver/CPLDDriver.h"
#include "../ManuLegend/ManuManager.h"

extern unsigned long micros();
extern volatile CPLDDriver* cpld;


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

const IMUState _state_indicies[] = {
  IMUState::STAGE_0,   // Undiscovered. Maybe absent.
  IMUState::STAGE_1,   // Discovered, but not init'd.
  IMUState::STAGE_2,   // Discovered and initiallized, but unknown register values.
  IMUState::STAGE_3,   // Fully initialized and sync'd. Un-calibrated.
  IMUState::STAGE_4,   // Calibrated and idle.
  IMUState::STAGE_5,   // Calibrated and reading.
  IMUState::FAULT,     // Fault.
  IMUState::UNDEF      // Not a state-machine value. A return code to simplifiy error-checks.
};


/*
* These are tables of frequencies versus periods (in micros). Lookup is faster
*   than calculation, typically.
*/
const UpdateRate2Hertz LSM9DS1::rate_settings_acc[MAXIMUM_RATE_INDEX_AG] = {
  {0.0,  0.0f},
  {14.9, (1/14.9f)},
  {59.5, (1/59.5f)},
  {119,  (1/119.0f)},
  {238,  (1/238.0f)},
  {476,  (1/476.0f)},
  {952,  (1/952.0f)}
};

const UpdateRate2Hertz LSM9DS1::rate_settings_gyr[MAXIMUM_RATE_INDEX_AG] = {
  {0.0,  0.0f},
  {10.0, (1/10.0f)},
  {50.0, (1/50.0f)},
  {119,  (1/119.0f)},
  {238,  (1/238.0f)},
  {476,  (1/476.0f)},
  {952,  (1/952.0f)}
};

const UpdateRate2Hertz LSM9DS1::rate_settings_mag[MAXIMUM_RATE_INDEX_MAG] = {
  {0.625, (1/0.625f)},
  {1.25,  (1/1.25f)},
  {2.5,   (1/2.5f)},
  {5.0,   (1/5.0f)},
  {10.0,  (1/10.0f)},
  {20.0,  (1/20.0f)},
  {40.0,  (1/40.0f)},
  {80.0,  (1/80.0f)}
};


/*
* These are generic table of scales versus unit-per-bit for 16-bit types.
* TODO: These need to be validated against actual datasheet values.
*/
const GainErrorMap LSM9DS1::error_map_acc[MAXIMUM_GAIN_INDEX_ACC] = {
  {2,  (2/32768.0f),  0.000030},
  {4,  (4/32768.0f),  0.000061},
  {6,  (6/32768.0f),  0.000092},
  {8,  (8/32768.0f),  0.000122},
  {16, (16/32768.0f), 0.000244}
};

const GainErrorMap LSM9DS1::error_map_gyr[MAXIMUM_GAIN_INDEX_GYR] = {
  {245,   (245/32768.0f),  0.00437 * 0.0174532777778},
  {500,   (500/32768.0f),  0.00875 * 0.0174532777778},
  {2000,  (2000/32768.0f), 0.03500 * 0.0174532777778}
};

const GainErrorMap LSM9DS1::error_map_mag[MAXIMUM_GAIN_INDEX_MAG] = {
  {4,  (4/32768.0f),  0.000061},
  {8,  (8/32768.0f),  0.000122},
  {12, (12/32768.0f), 0.00032},
  {16, (16/32768.0f), 0.000244}
};


/* The addresses of the registers named in the enum class RegID. */
const uint8_t _imu_address_map[] = {
  0x05, 0x07, 0x09,  // M: 16-bit offset registers
  0x0f, 0x20,
  0x21, 0x22, 0x23, 0x24, 0x27,
  0x28, 0x2a, 0x2c,  // M: 16-bit data registers
  0x30, 0x31,
  0x32,              // M: 16-bit threshold register
  // This is where the AG registers start.
  0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0a, 0x0b,
  0x0c, 0x0d, 0x0f, 0x10, 0x11, 0x12, 0x13, 0x14,
  0x15,              // I: 16-bit temperature register (11-bit)
  0x17,
  0x18, 0x1a, 0x1c,  // G: 16-bit gyro data registers
  0x1e, 0x1f, 0x20, 0x21, 0x22, 0x23, 0x24,
  0x26, 0x27,
  0x28, 0x2a, 0x2c,  // A: 16-bit acc data registers
  0x2e, 0x2f, 0x30,
  0x31, 0x33, 0x35,  // G: 16-bit threshold registers
  0x37
};

/* The default values of the registers named in the enum class RegID. */
// TODO: Make these match datasheet values.
const uint8_t _imu_reg_defaults[] = {
  0x00, 0x00, 0x00,  // M: 16-bit offset registers
  0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00,  // M: 16-bit data registers
  0x00, 0x00,
  0x00,              // M: 16-bit threshold register
  // This is where the AG registers start.
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00,              // I: 16-bit temperature register (11-bit)
  0x00,
  0x00, 0x00, 0x00,  // G: 16-bit gyro data registers
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00,
  0x00, 0x00, 0x00,  // A: 16-bit acc data registers
  0x00, 0x00, 0x00,
  0x00, 0x00, 0x00,  // G: 16-bit threshold registers
  0x00
};


/* The widths of the registers named in the enum class RegID. */
const uint8_t _imu_register_width_map[] = {
  2, 2, 2,  // M: 16-bit offset registers
  1, 1,
  1, 1, 1, 1, 1,
  2, 2, 2,  // M: 16-bit data registers
  1, 1,
  2,        // M: 16-bit threshold register
  // This is where the AG registers start.
  1, 1, 1, 1, 1, 1, 1, 1,
  1, 1, 1, 1, 1, 1, 1, 1,
  2,        // I: 16-bit temperature register (11-bit)
  1,
  2, 2, 2,  // G: 16-bit gyro data registers
  1, 1, 1, 1, 1, 1, 1,
  1, 1,
  2, 2, 2,  // A: 16-bit acc data registers
  1, 1, 1,
  2, 2, 2,  // G: 16-bit threshold registers
  1
};


/* The widths of the registers named in the enum class RegID. */
const bool _imu_register_writable_map[] = {
  true,  true,  true,   // M: 16-bit offset registers
  false, true,
  true,  true,  true,  true,  false,
  false, false, false,  // M: 16-bit data registers
  true,  false,
  true,                 // M: 16-bit threshold register
  // This is where the AG registers start.
  true,  true,  true,  true,  true,  true,  true,  true,
  true,  true,  false, true,  true,  true,  true,  false,
  false,                // I: 16-bit temperature register (11-bit)
  false,
  false, false, false,  // G: 16-bit gyro data registers
  true,  true,  true,  true,  true,  true,  true,
  false, false,
  false, false, false,  // A: 16-bit acc data registers
  true,  false, true,
  true,  true,  true,   // G: 16-bit threshold registers
  true
};


const uint8_t LSM9DS1::regAddr(RegID id) {
  return _imu_address_map[(uint8_t) id];
}

const uint8_t LSM9DS1::regWidth(RegID id) {
  return _imu_register_width_map[(uint8_t) id];
}

const bool LSM9DS1::regWritable(RegID id) {
  return _imu_register_writable_map[(uint8_t) id];
}


const float LSM9DS1::max_range_vect_acc  = 16.0;
const float LSM9DS1::max_range_vect_gyr  = 2000.0;
const float LSM9DS1::max_range_vect_mag  = 16.0;


/**
* Return an enumerator given the state index.
*
* @return enum State
*/
IMUState LSM9DS1::getStateByIndex(uint8_t state_idx) {
  if (state_idx < sizeof(_state_indicies)) {
    return _state_indicies[state_idx];
  }
  return IMUState::UNDEF;
}


/**
* Print a human-readable representation of the IMU fault condition.
*
* @return const char*
*/
const char* LSM9DS1::getErrorString(IMUFault fault_code) {
  switch (fault_code) {
    case IMUFault::NO_ERROR               :  return "NO_ERROR";
    case IMUFault::WRONG_IDENTITY         :  return "WRONG_IDENTITY";
    case IMUFault::INVALID_PARAM_ID       :  return "INVALID_PARAM_ID";
    case IMUFault::NOT_CALIBRATED         :  return "NOT_CALIBRATED";
    case IMUFault::NOT_WRITABLE           :  return "NOT_WRITABLE";
    case IMUFault::DATA_EXHAUSTED         :  return "DATA_EXHAUSTED";
    case IMUFault::NOT_INITIALIZED        :  return "NOT_INITIALIZED";
    case IMUFault::BUS_INSERTION_FAILED   :  return "INSERTION_FAILED";
    case IMUFault::BUS_OPERATION_FAILED_R :  return "OPERATION_FAILED_R";
    case IMUFault::BUS_OPERATION_FAILED_W :  return "OPERATION_FAILED_W";
    case IMUFault::REGISTER_UNDEFINED     :  return "REGISTER_UNDEFINED";
  }
  return "<UNKNOWN>";
}


/**
* Print a human-readable representation of the IMU state.
*
* @return const char*
*/
const char* LSM9DS1::getStateString(IMUState state) {
  switch (state) {
    case IMUState::UNDEF:    return "UNDEF";
    case IMUState::FAULT:    return "FAULT";
    case IMUState::STAGE_0:  return "STAGE_0";
    case IMUState::STAGE_1:  return "STAGE_1";
    case IMUState::STAGE_2:  return "STAGE_2";
    case IMUState::STAGE_3:  return "STAGE_3";
    case IMUState::STAGE_4:  return "STAGE_4";
    case IMUState::STAGE_5:  return "STAGE_5";
  }
  return "<UNKNOWN>";
}


/**
* Print the IMU register name.
*
* @return const char*
*/
const char* LSM9DS1::regNameString(RegID idx) {
  switch (idx) {
    case RegID::M_OFFSET_X:          return "M_OFFSET_X";
    case RegID::M_OFFSET_Y:          return "M_OFFSET_Y";
    case RegID::M_OFFSET_Z:          return "M_OFFSET_Z";
    case RegID::M_WHO_AM_I:          return "M_WHO_AM_I";
    case RegID::M_CTRL_REG1:         return "M_CTRL_REG1";
    case RegID::M_CTRL_REG2:         return "M_CTRL_REG2";
    case RegID::M_CTRL_REG3:         return "M_CTRL_REG3";
    case RegID::M_CTRL_REG4:         return "M_CTRL_REG4";
    case RegID::M_CTRL_REG5:         return "M_CTRL_REG5";
    case RegID::M_STATUS_REG:        return "M_STATUS_REG";
    case RegID::M_DATA_X:            return "M_DATA_X";
    case RegID::M_DATA_Y:            return "M_DATA_Y";
    case RegID::M_DATA_Z:            return "M_DATA_Z";
    case RegID::M_INT_CFG:           return "M_INT_CFG";
    case RegID::M_INT_SRC:           return "M_INT_SRC";
    case RegID::M_INT_TSH:           return "M_INT_TSH";
    case RegID::AG_ACT_THS:          return "AG_ACT_THS";
    case RegID::AG_ACT_DUR:          return "AG_ACT_DUR";
    case RegID::A_INT_GEN_CFG:       return "A_INT_GEN_CFG";
    case RegID::A_INT_GEN_THS_X:     return "A_INT_GEN_THS_X";
    case RegID::A_INT_GEN_THS_Y:     return "A_INT_GEN_THS_Y";
    case RegID::A_INT_GEN_THS_Z:     return "A_INT_GEN_THS_Z";
    case RegID::A_INT_GEN_DURATION:  return "A_INT_GEN_DURATION";
    case RegID::G_REFERENCE:         return "G_REFERENCE";
    case RegID::AG_INT1_CTRL:        return "AG_INT1_CTRL";
    case RegID::AG_INT2_CTRL:        return "AG_INT2_CTRL";
    case RegID::AG_WHO_AM_I:         return "AG_WHO_AM_I";
    case RegID::G_CTRL_REG1:         return "G_CTRL_REG1";
    case RegID::G_CTRL_REG2:         return "G_CTRL_REG2";
    case RegID::G_CTRL_REG3:         return "G_CTRL_REG3";
    case RegID::G_ORIENT_CFG:        return "G_ORIENT_CFG";
    case RegID::G_INT_GEN_SRC:       return "G_INT_GEN_SRC";
    case RegID::AG_DATA_TEMP:        return "AG_DATA_TEMP";
    case RegID::AG_STATUS_REG:       return "AG_STATUS_REG";
    case RegID::G_DATA_X:            return "G_DATA_X";
    case RegID::G_DATA_Y:            return "G_DATA_Y";
    case RegID::G_DATA_Z:            return "G_DATA_Z";
    case RegID::AG_CTRL_REG4:        return "AG_CTRL_REG4";
    case RegID::A_CTRL_REG5:         return "A_CTRL_REG5";
    case RegID::A_CTRL_REG6:         return "A_CTRL_REG6";
    case RegID::A_CTRL_REG7:         return "A_CTRL_REG7";
    case RegID::AG_CTRL_REG8:        return "AG_CTRL_REG8";
    case RegID::AG_CTRL_REG9:        return "AG_CTRL_REG9";
    case RegID::AG_CTRL_REG10:       return "AG_CTRL_REG10";
    case RegID::A_INT_GEN_SRC:       return "A_INT_GEN_SRC";
    case RegID::AG_STATUS_REG_ALT:   return "AG_STATUS_REG_ALT";
    case RegID::A_DATA_X:            return "A_DATA_X";
    case RegID::A_DATA_Y:            return "A_DATA_Y";
    case RegID::A_DATA_Z:            return "A_DATA_Z";
    case RegID::AG_FIFO_CTRL:        return "AG_FIFO_CTRL";
    case RegID::AG_FIFO_SRC:         return "AG_FIFO_SRC";
    case RegID::G_INT_GEN_CFG:       return "G_INT_GEN_CFG";
    case RegID::G_INT_GEN_THS_X:     return "G_INT_GEN_THS_X";
    case RegID::G_INT_GEN_THS_Y:     return "G_INT_GEN_THS_Y";
    case RegID::G_INT_GEN_THS_Z:     return "G_INT_GEN_THS_Z";
    case RegID::G_INT_GEN_DURATION:  return "G_INT_GEN_DURATIO";
  }
  return "<UNKNOWN>";
}


RegID RegPtrMap::regIdFromAddr(uint8_t addr) {
  switch (addr & 0x7F) {
    // TODO: Fill with real values.
    case 0:    return RegID::M_WHO_AM_I;
    case 1:    return RegID::M_WHO_AM_I;
    case 2:    return RegID::M_WHO_AM_I;
    case 3:    return RegID::M_WHO_AM_I;
    case 4:    return RegID::M_WHO_AM_I;
    case 5:    return RegID::M_WHO_AM_I;
    case 6:    return RegID::M_WHO_AM_I;
    case 7:    return RegID::M_WHO_AM_I;
    case 8:    return RegID::M_WHO_AM_I;
    case 9:    return RegID::M_WHO_AM_I;
    case 10:   return RegID::M_WHO_AM_I;
    case 11:   return RegID::M_WHO_AM_I;
    case 12:   return RegID::M_WHO_AM_I;
    case 13:   return RegID::M_WHO_AM_I;
    case 14:   return RegID::M_WHO_AM_I;
    case 15:   return RegID::M_WHO_AM_I;
    case 16:   return RegID::M_WHO_AM_I;
    case 17:   return RegID::M_WHO_AM_I;
    case 18:   return RegID::M_WHO_AM_I;
    case 19:   return RegID::M_WHO_AM_I;
    case 20:   return RegID::M_WHO_AM_I;
    case 21:   return RegID::M_WHO_AM_I;
    case 22:   return RegID::M_WHO_AM_I;
    case 23:   return RegID::M_WHO_AM_I;
    case 24:   return RegID::M_WHO_AM_I;
    case 25:   return RegID::M_WHO_AM_I;
    case 26:   return RegID::M_WHO_AM_I;
    case 27:   return RegID::M_WHO_AM_I;
    case 28:   return RegID::M_WHO_AM_I;
    case 29:   return RegID::M_WHO_AM_I;
    case 30:   return RegID::M_WHO_AM_I;
    case 31:   return RegID::M_WHO_AM_I;
    case 32:   return RegID::M_WHO_AM_I;
    case 33:   return RegID::M_WHO_AM_I;
    case 34:   return RegID::M_WHO_AM_I;
    case 35:   return RegID::M_WHO_AM_I;
    case 36:   return RegID::M_WHO_AM_I;
    case 37:   return RegID::M_WHO_AM_I;
    case 38:   return RegID::M_WHO_AM_I;
    case 39:   return RegID::M_WHO_AM_I;
    case 40:   return RegID::M_WHO_AM_I;
    case 41:   return RegID::M_WHO_AM_I;
    case 42:   return RegID::M_WHO_AM_I;
    case 43:   return RegID::M_WHO_AM_I;
    case 44:   return RegID::M_WHO_AM_I;
    case 45:   return RegID::M_WHO_AM_I;
    case 46:   return RegID::M_WHO_AM_I;
    case 47:   return RegID::M_WHO_AM_I;
    case 48:   return RegID::M_WHO_AM_I;
    case 49:   return RegID::M_WHO_AM_I;
    case 50:   return RegID::M_WHO_AM_I;
    case 51:   return RegID::M_WHO_AM_I;
    case 52:   return RegID::M_WHO_AM_I;
    case 53:   return RegID::M_WHO_AM_I;
    case 54:   return RegID::M_WHO_AM_I;
    case 55:   return RegID::M_WHO_AM_I;
  }
}



/*******************************************************************************
*   ___ _              ___      _ _              _      _
*  / __| |__ _ ______ | _ ) ___(_) |___ _ _ _ __| |__ _| |_ ___
* | (__| / _` (_-<_-< | _ \/ _ \ | / -_) '_| '_ \ / _` |  _/ -_)
*  \___|_\__,_/__/__/ |___/\___/_|_\___|_| | .__/_\__,_|\__\___|
*                                          |_|
* Constructors/destructors, class initialization functions and so-forth...
*******************************************************************************/
//TODO: I'll work back up to this once it isn't in the way.
//LSM9DS1::LSM9DS1(uint8_t addr, uint8_t ident_idx, uint8_t idx_test_0, uint8_t idx_test_1) :
//  IDX_T0(idx_test_0), IDX_T1(idx_test_1), BUS_ADDR(addr), IDX_ID(ident_idx)
//{
//  init();
//}

LSM9DS1::LSM9DS1(const RegPtrMap* pm) : _ptr_map(pm) {
}


LSM9DS1::~LSM9DS1() {
}


void LSM9DS1::class_init(uint8_t address) {
  last_val_mag(0.0f, 0.0f, 0.0f);
  last_val_acc(0.0f, 0.0f, 0.0f);
  last_val_gyr(0.0f, 0.0f, 0.0f);
  noise_floor_mag(0, 0, 0);
  noise_floor_acc(0, 0, 0);
  noise_floor_gyr(0, 0, 0);

  // Local class stuff...
  scale_mag           = 0;
  update_rate_mag     = 0;
  discards_remain_mag = 0;
  discards_total_mag  = 0;

  scale_acc           = 0;
  update_rate_acc     = 0;
  discards_remain_acc = 0;
  discards_total_acc  = 0;

  scale_gyr           = 0;
  update_rate_gyr     = 0;
  discards_remain_gyr = 0;
  discards_total_gyr  = 0;

  BUS_ADDR = address;
  IDX_T0 = RegID::M_OFFSET_X;
  IDX_T1 = RegID::M_OFFSET_Y;
  IDX_ID = RegID::M_WHO_AM_I;
  init();
}


/**
* Called to init the common boilerplate for this sensor.
*
* @return  IMUFault::NO_ERROR or appropriate failure code.
*/
IMUFault LSM9DS1::init() {
  // Force our states back to reset.
  imu_state          = IMUState::STAGE_0;
  desired_state      = IMUState::STAGE_0;
  error_condition    = IMUFault::NO_ERROR;

  _imu_flags = 1;
  sample_count       = 0;
  time_stamp_base    = 0;
  if (pending_samples) {
    *pending_samples = 0;
  }
  return IMUFault::NO_ERROR;
}


/**
* Calling this function will reset the common class elements to their
*   default states.
*/
void LSM9DS1::reset() {
  scale_mag           = 0;
  update_rate_mag     = 0;
  discards_remain_mag = 0;
  discards_total_mag  = 0;

  scale_acc           = 0;
  update_rate_acc     = 0;
  discards_remain_acc = 0;
  discards_total_acc  = 0;

  scale_gyr           = 0;
  update_rate_gyr     = 0;
  discards_remain_gyr = 0;
  discards_total_gyr  = 0;

  noise_floor_mag.set(0.0f, 0.0f, 0.0f);
  noise_floor_acc.set(0.0f, 0.0f, 0.0f);
  noise_floor_gyr.set(0.0f, 0.0f, 0.0f);

  // TODO: Blow away our idea of what is in the registers.
  // mark_it_zero();
  init();
  writeRegister(RegID::AG_CTRL_REG8, 0x01);
  writeRegister(RegID::M_CTRL_REG2, 0x04);
}



IMUFault LSM9DS1::identity_check() {
  return (present() ? IMUFault::NO_ERROR : IMUFault::WRONG_IDENTITY);
}


int8_t LSM9DS1::readSensor() {
  int8_t return_value = 0;
  if (!present()) {
    return -1;
  }
  if (!calibrated()) {
    //return IMUFault::NOT_CALIBRATED;
  }

  if (initComplete()) {
  }
  return return_value;
}


/**
* Calling this will cause us to generate two random bytes and write them to two
*   separate registers chosen by the extending class. Those writes (if successful)
*   should cause an automatic re-read of those same registers to check that the bytes
*   were written.
*/
void LSM9DS1::write_test_bytes() {
  io_test_val_0 = (uint8_t) randomInt() % 128;
  io_test_val_1 = (uint8_t) randomInt() % 128;

  writeRegister(IDX_T0, io_test_val_0);
  writeRegister(IDX_T1, io_test_val_1);
}


// Init parameters for the magnetometer. Starting at CTRL_REG1.
uint8_t bulk_init_block_m[5] = {
  0b11011000,  // Temp compensated, mid-range X/Y performance, 40Hz output.
  0b00000000,  // 4 gauss,
  0b00000000,  // SPI read/write, continuous conversion. Normal power mode.
  0b00001000,  // Mid-range Z-axis performance, little-endian,
  0b00000000,  // Continuous (unblocked) data register update.
};

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

// Init parameters for the inertial aspect. Starting at RegID::G_INT_GEN_CFG.
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


/*
* For now:
* 0: Off  (Implies other register activity)
* 1: Lowest rate while still collecting data.
* 2: Low-accuracy rate. ~100Hz from each FIFO'd sensor.
* 3: Moderate rate.
* 4: Highest rate supported.
*/
void LSM9DS1::setSampleRateProfile(uint8_t profile_idx) {
  switch (profile_idx) {
    case 0:
      //delta_t = 0.0f;
      set_sample_rate_acc(0);
      set_sample_rate_gyr(0);
      set_sample_rate_mag(0);
      break;
    case 1:
      //delta_t = 0.0f;
      set_sample_rate_acc(1);
      set_sample_rate_gyr(1);
      set_sample_rate_mag(1);
      break;
    case 2:
      //delta_t = 0.0f;
      set_sample_rate_acc(6);
      set_sample_rate_gyr(2);
      set_sample_rate_mag(5);
      break;
    case 3:
      //delta_t = 0.0f;
      set_sample_rate_acc(8);
      set_sample_rate_gyr(3);
      set_sample_rate_mag(5);
      break;
    case 4:
      //delta_t = 0.0f;
      set_sample_rate_acc(10);
      set_sample_rate_gyr(4);
      set_sample_rate_mag(6);
      break;
    default:
      break;
  }
}


/*
*
*/
bool LSM9DS1::is_setup_completed() {
  if (!present()) return false;

  if (!initComplete()) {   // TODO: Redundant.
    if (initPending()) {   // TODO: Redundant.
      //if (reg_defs[RegID::M_CTRL_REG1].dirty) return false;
      //if (reg_defs[RegID::M_CTRL_REG5].dirty) return false;
      //if (reg_defs[RegID::G_INT_GEN_CFG].dirty) return false;
      //if (reg_defs[RegID::AG_FIFO_CTRL].dirty) return false;
    }
  }

  return true;
}


/**
* Our purpose here is to verify that our test value comes back on a read. This is a
*   bus-integrity test that must be passed prior to entering INIT-3.
*
* @return true if the test passes. False otherwise.
*/
bool LSM9DS1::integrity_check() {
  if (!present()) return false;

  // If we are ain a state where we are reading the init values back, look for our test
  // values, and fail the init if they are not found.
  if (io_test_val_0 == regValue(IDX_T0)) {
    if (io_test_val_1 == regValue(IDX_T1)) {
        // We will call this successful init.
        if (getVerbosity() > 5) {
          StringBuilder local_log;
          local_log.concat("Successful readback!");
          Kernel::log(&local_log);
        }
        // Rewrite valid values to those registers if necessary.
        //writeDirtyRegisters();
        _alter_flags(true, IMU_COMMON_FLAG_HW_WRITABLE);
        return true;
    }
    else {
      if (getVerbosity() > 2) {
        StringBuilder local_log;
        local_log.concatf("Failed integrity check (index 0x%02x). Found 0x%02x. Expected 0x%02x.\n", IDX_T1, regValue(IDX_T1), io_test_val_1);
        Kernel::log(&local_log);
      }
    }
  }
  else {
    if (getVerbosity() > 2) {
      StringBuilder local_log;
      local_log.concatf("Failed integrity check (index 0x%02x). Found 0x%02x. Expected 0x%02x.\n", IDX_T0, regValue(IDX_T0), io_test_val_0);
      Kernel::log(&local_log);
    }
  }

  error_condition = IMUFault::NOT_WRITABLE;
  return false;
}



/**
* Call to set the desired IMU state. The class should then take whatever action is needed
*   to make reality match. This will be asynchronously done, so the caller will probably need
*   to be informed of the change after it has been completed, or can go no further.
*
* @param  uint8_t The new state desired by the firmware (the Legend).
* @return non-zero on error.
*/
IMUFault LSM9DS1::setDesiredState(IMUState nu) {
  if (present() && (nu < IMUState::STAGE_1)) {
    // If we already know the sensor is there, why go back further than this?
    local_log.concat("Trying to move to a state lower than allowed.\n");
    Kernel::log(&local_log);
    return IMUFault::INVALID_PARAM_ID;
  }

  if (desired_state != nu) {
    if (!desired_state_attained()) {
      // TODO
      // The IMU is not at equilibrium. It may be ok to change the desired stage as long as we don't have
      //   bus operations pending.
      if (getVerbosity() > 2) {
        //local_log.concatf("%s tried to move to state %s while the IMU is off-balance (%s --> %s). Rejecting request.\n", imu_type(), getStateString(nu), getStateString(), getStateString(desired_state));
        local_log.concatf("Tried to move to state %s while the IMU is off-balance (%s --> %s). We will allow this for now.\n", getStateString(nu), getStateString(), getStateString(desired_state));
        Kernel::log(&local_log);
      }
      //return -2;
    }

    desired_state = nu;
    step_state();
  }
  return IMUFault::NO_ERROR;
}


/**
* Assumes that the operation prior set the state to whatever is current.
*
* @return true if the state is stable, and the integrator should be notified.
*/
bool LSM9DS1::step_state() {
  if (!desired_state_attained()) {
    if (IMUFault::NO_ERROR != error_condition) {
      // We shouldn't be changing states if there is an error condition.
      // Reset is the only way to exit the condition at present.
      if (getVerbosity() > 2) {
        local_log.concatf("Step_state() was called while we are in an error condition: %s\n",  getErrorString());
        Kernel::log(&local_log);
      }
      return true;
    }

    switch (getState()) {
      case IMUState::STAGE_0:  // We think the IIU might be physicaly absent.
        //reset(); ?
        identity_check();
        break;

      case IMUState::STAGE_1:  // We are sure the IMU is present, but we haven't done anything with it.
        //configure_sensor();
        break;

      case IMUState::STAGE_2:  // Discovered and initiallized, but unknown register values.
        if (is_setup_completed()) {
          //bulk_refresh();
        }
        else {
          set_state(IMUState::STAGE_1);
          return true;
        }
        break;

      case IMUState::STAGE_3:  // Fully initialized and sync'd. Un-calibrated.
        sb_next_write = 0;
        //readSensor();
        // TODO: Stop skipping calibrate().
        break;

      case IMUState::STAGE_4:  // Calibrated and idle.
        if (desiredState() == IMUState::STAGE_5) {
          // Enable the chained reads, and start the process rolling.
          //readSensor();
        }
        else {
          // Downgrading to init state 3 (recalibrate).
          set_state(IMUState::STAGE_3);
          sb_next_write = 0;
          //readSensor();
          return true;
        }
        break;

      case IMUState::STAGE_5:  // Calibrated and reading.
        switch (desiredState()) {
          case IMUState::STAGE_4:   // Stop reads.
            set_state(IMUState::STAGE_4);
            return false;   /// Note the slight break from convention... Careful...

          case IMUState::STAGE_3:  // Downgrading to init state 3 (recalibrate).
            set_state(IMUState::STAGE_3);
            sb_next_write = 0;
            //readSensor();
            return true;
          case IMUState::STAGE_5:  // Keep reading.
            return true;
          default:
            break;
        }
        break;

      default:
        break;
    }
    return false;
  }
  return true;
}


IMUFault LSM9DS1::writeRegister(RegID idx, unsigned int nu_val) {
  const uint8_t* ptr = _ptr_map->regPtr(idx);
  if (ptr) {
    if (regWritable(idx)) {
      switch (regWidth(idx)) {
        // These are the only two widths in the sensor.
        case 2:
          *((uint16_t*)ptr) = (uint16_t)nu_val;
          break;
        case 1:
          *((uint8_t*) ptr) = (uint8_t) nu_val;
          break;
        default:
          return IMUFault::NOT_WRITABLE;
      }
      return IMUFault::NO_ERROR;
    }
    return IMUFault::NOT_WRITABLE;
  }
  return IMUFault::REGISTER_UNDEFINED;
}


/*
* Convenience fxn. Returns 0 if register index is out of bounds.
*/
unsigned int LSM9DS1::regValue(RegID idx) {
  const uint8_t* ptr = _ptr_map->regPtr(idx);
  if (ptr) {
    switch (regWidth(idx)) {
      // These are the only two widths in the sensor.
      case 2: return *((uint16_t*)ptr);
      case 1: return *((uint8_t*) ptr);
    }
  }
  return 0;
}


/*
* Looks at our local offset table to obtain value. This was set at instantiation
*   by ManaManager.
*/
const uint8_t* RegPtrMap::regPtr(RegID idx) const {
  switch (idx) {
    case RegID::M_OFFSET_X:          return nullptr;
    case RegID::M_OFFSET_Y:          return nullptr;
    case RegID::M_OFFSET_Z:          return nullptr;
    case RegID::M_WHO_AM_I:          return nullptr;
    case RegID::M_CTRL_REG1:         return nullptr;
    case RegID::M_CTRL_REG2:         return nullptr;
    case RegID::M_CTRL_REG3:         return nullptr;
    case RegID::M_CTRL_REG4:         return nullptr;
    case RegID::M_CTRL_REG5:         return nullptr;
    case RegID::M_STATUS_REG:        return nullptr;
    case RegID::M_DATA_X:            return nullptr;
    case RegID::M_DATA_Y:            return nullptr;
    case RegID::M_DATA_Z:            return nullptr;
    case RegID::M_INT_CFG:           return nullptr;
    case RegID::M_INT_SRC:           return nullptr;
    case RegID::M_INT_TSH:           return nullptr;
    case RegID::AG_ACT_THS:          return AG_ACT+0;
    case RegID::AG_ACT_DUR:          return AG_ACT+1;
    case RegID::A_INT_GEN_CFG:       return AG_BLOCK_0+0;
    case RegID::A_INT_GEN_THS_X:     return AG_BLOCK_0+1;
    case RegID::A_INT_GEN_THS_Y:     return AG_BLOCK_0+2;
    case RegID::A_INT_GEN_THS_Z:     return AG_BLOCK_0+3;
    case RegID::A_INT_GEN_DURATION:  return AG_BLOCK_0+4;
    case RegID::G_REFERENCE:         return AG_BLOCK_0+5;
    case RegID::AG_INT1_CTRL:        return nullptr;
    case RegID::AG_INT2_CTRL:        return nullptr;
    case RegID::AG_WHO_AM_I:         return nullptr;
    case RegID::G_CTRL_REG1:         return AG_CTRL1_3+0;
    case RegID::G_CTRL_REG2:         return AG_CTRL1_3+1;
    case RegID::G_CTRL_REG3:         return AG_CTRL1_3+2;
    case RegID::G_ORIENT_CFG:        return nullptr;
    case RegID::G_INT_GEN_SRC:       return nullptr;
    case RegID::AG_DATA_TEMP:        return nullptr;
    case RegID::AG_STATUS_REG:       return nullptr;
    case RegID::G_DATA_X:            return nullptr;
    case RegID::G_DATA_Y:            return nullptr;
    case RegID::G_DATA_Z:            return nullptr;
    case RegID::AG_CTRL_REG4:        return nullptr;
    case RegID::A_CTRL_REG5:         return nullptr;
    case RegID::A_CTRL_REG6:         return AG_CTRL6_7+0;
    case RegID::A_CTRL_REG7:         return AG_CTRL6_7+1;
    case RegID::AG_CTRL_REG8:        return nullptr;
    case RegID::AG_CTRL_REG9:        return nullptr;
    case RegID::AG_CTRL_REG10:       return nullptr;
    case RegID::A_INT_GEN_SRC:       return nullptr;
    case RegID::AG_STATUS_REG_ALT:   return nullptr;
    case RegID::A_DATA_X:            return nullptr;
    case RegID::A_DATA_Y:            return nullptr;
    case RegID::A_DATA_Z:            return nullptr;
    case RegID::AG_FIFO_CTRL:        return nullptr;
    case RegID::AG_FIFO_SRC:         return nullptr;
    case RegID::G_INT_GEN_CFG:       return nullptr;
    case RegID::G_INT_GEN_THS_X:     return nullptr;
    case RegID::G_INT_GEN_THS_Y:     return nullptr;
    case RegID::G_INT_GEN_THS_Z:     return nullptr;
    case RegID::G_INT_GEN_DURATION:  return nullptr;
  }
  return nullptr;
}


/**
* Dump the contents of this device to the logger.
*
* @param   StringBuilder* The buffer into which this fxn should write its output.
*/
void LSM9DS1::dumpDevRegs(StringBuilder *output) {
  output->concatf("\n-------------------------------------------------------\n--- IMU 0x%04x  %s ==>  %s \n-------------------------------------------------------\n", BUS_ADDR, getStateString(imu_state), (desired_state_attained() ? "STABLE" : getStateString(desired_state)));
  output->concatf("--- sample_count        %d\n--- pending_samples     %d\n\n", sample_count, *pending_samples);
  if (getVerbosity() > 1) {
    output->concatf("--- calibration smpls   %d\n", sb_next_write);
    output->concatf("--- Base filter param   %d\n", base_filter_param);
  }
  output->concatf("--- Error condition     %s\n---\n", getErrorString(error_condition));

  if (getVerbosity() > 1) {
    output->concatf("--- update_rate_mag     %3.0f Hz\n", (double) rate_settings_mag[update_rate_mag].hertz);
    output->concatf("--- update_rate_acc     %3.0f Hz\n", (double) rate_settings_acc[update_rate_acc].hertz);
    output->concatf("--- update_rate_gyr     %3.0f Hz\n", (double) rate_settings_gyr[update_rate_gyr].hertz);
  }
  if (getVerbosity() > 2) {
    output->concatf("--- scale_mag           +/-%d gauss\n", error_map_mag[scale_mag].scale);
    output->concatf("--- scale_acc           +/-%d m/s\n", error_map_acc[scale_acc].scale);
    output->concatf("--- scale_gyr           +/-%d deg/s\n", error_map_gyr[scale_gyr].scale);
    output->concatf("--- autoscale_mag       %s\n", (autoscale_mag() ? "yes" : "no"));
    output->concatf("--- autoscale_acc       %s\n", (autoscale_acc() ? "yes" : "no"));
    output->concatf("--- autoscale_gyr       %s\n", (autoscale_gyr() ? "yes" : "no"));
    output->concatf("--- noise_floor_mag     (%d, %d, %d)\n", noise_floor_mag.x, noise_floor_mag.y, noise_floor_mag.z);
    output->concatf("--- noise_floor_acc     (%d, %d, %d)\n", noise_floor_acc.x, noise_floor_acc.y, noise_floor_acc.z);
    output->concatf("--- noise_floor_gyr     (%d, %d, %d)\n", noise_floor_gyr.x, noise_floor_gyr.y, noise_floor_gyr.z);
  }
}


/**
* When a bus operation completes, it is passed back to its issuing class.
*
* @param  _op  The bus operation that was completed.
* @return SPI_CALLBACK_NOMINAL on success, or appropriate error code.
*/
int8_t LSM9DS1::io_op_callback(BusOp* _op) {
  SPIBusOp* op = (SPIBusOp*) _op;
  int8_t return_value = SPI_CALLBACK_NOMINAL;

  // There is zero chance this object will be a null pointer unless it was done on purpose.
  if (op->hasFault()) {
    if (getVerbosity() > 3) {
      local_log.concat("~~~~~~~~LSM9DS1::io_op_callback (ERROR CASE -1)\n");
      op->printDebug(&local_log);
      Kernel::log(&local_log);
    }
    error_condition = (BusOpcode::RX == op->get_opcode()) ? IMUFault::BUS_OPERATION_FAILED_R : IMUFault::BUS_OPERATION_FAILED_W;

    // TODO: Should think carefully, and...   return_value = SPI_CALLBACK_RECYCLE;   // Re-run the job.
    return SPI_CALLBACK_ERROR;
  }

  //TODO: Switch/case for ag/mag register.
  RegID idx = RegPtrMap::regIdFromAddr(op->getTransferParam(3));
  //unsigned int val =

  if (true) {  // TODO: Horrid. Wrong.
    error_condition = (BusOpcode::RX == op->get_opcode()) ? io_op_callback_mag_read(idx, 0) : io_op_callback_mag_write(idx, 0);
  }
  else {
    error_condition = (BusOpcode::RX == op->get_opcode()) ? io_op_callback_ag_read(idx, 0) : io_op_callback_ag_write(idx, 0);
  }

  if (local_log.length() > 0) Kernel::log(&local_log);
  return return_value;
}
