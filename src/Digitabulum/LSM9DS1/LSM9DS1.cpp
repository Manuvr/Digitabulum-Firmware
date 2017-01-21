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

extern unsigned long micros(void);
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


const float LSM9DS1::max_range_vect_acc  = 16.0;
const float LSM9DS1::max_range_vect_gyr  = 2000.0;
const float LSM9DS1::max_range_vect_mag   = 16.0;


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

LSM9DS1::LSM9DS1() {
}


LSM9DS1::~LSM9DS1() {
}


void LSM9DS1::class_init(uint8_t address) {
  // First, we should define our registers....
  // Mag has 16 registers. 23 bytes...
  // AG has 40 registers.

  // Now we should give them initial definitions. This is our chance to set default configs.
  // Any config which we want written during init() should have dirty set to true.
  //reg_defs[LSM9DS1_M_OFFSET_X]      = DeviceRegister((BUS_ADDR + 0x05), (uint16_t) 0x00, (register_pool +  0), false, false, true);
  //reg_defs[LSM9DS1_M_OFFSET_Y]      = DeviceRegister((BUS_ADDR + 0x07), (uint16_t) 0x00, (register_pool +  2), false, false, true);
  //reg_defs[LSM9DS1_M_OFFSET_Z]      = DeviceRegister((BUS_ADDR + 0x09), (uint16_t) 0x00, (register_pool +  4), false, false, true);
  //reg_defs[LSM9DS1_M_WHO_AM_I]      = DeviceRegister((BUS_ADDR + 0x0F), (uint8_t)  0x00, (register_pool +  6), false, false, false);
  //reg_defs[LSM9DS1_M_CTRL_REG1]     = DeviceRegister((BUS_ADDR + 0x20), (uint8_t)  0x10, (register_pool +  7), false, false, true );  //
  //reg_defs[LSM9DS1_M_CTRL_REG2]     = DeviceRegister((BUS_ADDR + 0x21), (uint8_t)  0x00, (register_pool +  8), false, false, true );  //
  //reg_defs[LSM9DS1_M_CTRL_REG3]     = DeviceRegister((BUS_ADDR + 0x22), (uint8_t)  0x03, (register_pool +  9), false, false, true );  //
  //reg_defs[LSM9DS1_M_CTRL_REG4]     = DeviceRegister((BUS_ADDR + 0x23), (uint8_t)  0x00, (register_pool + 10), false, false, true );  //
  //reg_defs[LSM9DS1_M_CTRL_REG5]     = DeviceRegister((BUS_ADDR + 0x24), (uint8_t)  0x00, (register_pool + 11), false, false, true );  //
  //reg_defs[LSM9DS1_M_STATUS_REG]    = DeviceRegister((BUS_ADDR + 0x27), (uint8_t)  0x00, (register_pool + 12), false, false, true );  //
  //reg_defs[LSM9DS1_M_DATA_X]        = DeviceRegister((BUS_ADDR + 0x28), (uint16_t) 0x00, (register_pool + 13), false, false, false);
  //reg_defs[LSM9DS1_M_DATA_Y]        = DeviceRegister((BUS_ADDR + 0x2A), (uint16_t) 0x00, (register_pool + 15), false, false, false);
  //reg_defs[LSM9DS1_M_DATA_Z]        = DeviceRegister((BUS_ADDR + 0x2C), (uint16_t) 0x00, (register_pool + 17), false, false, false);
  //reg_defs[LSM9DS1_M_INT_CFG]       = DeviceRegister((BUS_ADDR + 0x30), (uint8_t)  0x08, (register_pool + 19), false, false, true);
  //reg_defs[LSM9DS1_M_INT_SRC]       = DeviceRegister((BUS_ADDR + 0x31), (uint8_t)  0x00, (register_pool + 20), false, false, false);
  //reg_defs[LSM9DS1_M_INT_TSH]       = DeviceRegister((BUS_ADDR + 0x32), (uint16_t) 0x00, (register_pool + 21), false, false, true);

  //reg_defs[LSM9DS1_AG_ACT_THS]         = DeviceRegister((BUS_ADDR + 0x04), (uint8_t)  0x00, (register_pool +  0), false, false, true);
  //reg_defs[LSM9DS1_AG_ACT_DUR]         = DeviceRegister((BUS_ADDR + 0x05), (uint8_t)  0x00, (register_pool +  1), false, false, true);
  //reg_defs[LSM9DS1_A_INT_GEN_CFG]      = DeviceRegister((BUS_ADDR + 0x06), (uint8_t)  0x00, (register_pool +  2), false, false, true);
  //reg_defs[LSM9DS1_A_INT_GEN_THS_X]    = DeviceRegister((BUS_ADDR + 0x07), (uint8_t)  0x00, (register_pool +  3), false, false, true);
  //reg_defs[LSM9DS1_A_INT_GEN_THS_Y]    = DeviceRegister((BUS_ADDR + 0x08), (uint8_t)  0x00, (register_pool +  4), false, false, true);
  //reg_defs[LSM9DS1_A_INT_GEN_THS_Z]    = DeviceRegister((BUS_ADDR + 0x09), (uint8_t)  0x00, (register_pool +  5), false, false, true);
  //reg_defs[LSM9DS1_A_INT_GEN_DURATION] = DeviceRegister((BUS_ADDR + 0x0A), (uint8_t)  0x00, (register_pool +  6), false, false, true);
  //reg_defs[LSM9DS1_G_REFERENCE]        = DeviceRegister((BUS_ADDR + 0x0B), (uint8_t)  0x00, (register_pool +  7), false, false, true);
  //reg_defs[LSM9DS1_AG_INT1_CTRL]       = DeviceRegister((BUS_ADDR + 0x0C), (uint8_t)  0x00, (register_pool +  8), false, false, true);
  //reg_defs[LSM9DS1_AG_INT2_CTRL]       = DeviceRegister((BUS_ADDR + 0x0D), (uint8_t)  0x00, (register_pool +  9), false, false, true);
  //reg_defs[LSM9DS1_AG_WHO_AM_I]        = DeviceRegister((BUS_ADDR + 0x0F), (uint8_t)  0x00, (register_pool + 10), false, false, true);
  //reg_defs[LSM9DS1_G_CTRL_REG1]        = DeviceRegister((BUS_ADDR + 0x10), (uint8_t)  0x00, (register_pool + 11), false, false, true);
  //reg_defs[LSM9DS1_G_CTRL_REG2]        = DeviceRegister((BUS_ADDR + 0x11), (uint8_t)  0x00, (register_pool + 12), false, false, true);
  //reg_defs[LSM9DS1_G_CTRL_REG3]        = DeviceRegister((BUS_ADDR + 0x12), (uint8_t)  0x00, (register_pool + 13), false, false, true);
  //reg_defs[LSM9DS1_G_ORIENT_CFG]       = DeviceRegister((BUS_ADDR + 0x13), (uint8_t)  0x00, (register_pool + 14), false, false, true);
  //reg_defs[LSM9DS1_G_INT_GEN_SRC]      = DeviceRegister((BUS_ADDR + 0x14), (uint8_t)  0x00, (register_pool + 15), false, false, true);
  //reg_defs[LSM9DS1_AG_DATA_TEMP]       = DeviceRegister((BUS_ADDR + 0x15), (uint16_t) 0x00, (register_pool + 16), false, false, true);
  //reg_defs[LSM9DS1_AG_STATUS_REG]      = DeviceRegister((BUS_ADDR + 0x17), (uint8_t)  0x00, (register_pool + 18), false, false, true);
  //reg_defs[LSM9DS1_G_DATA_X]           = DeviceRegister((BUS_ADDR + 0x18), (uint16_t) 0x00, (register_pool + 19), false, false, true);
  //reg_defs[LSM9DS1_G_DATA_Y]           = DeviceRegister((BUS_ADDR + 0x1A), (uint16_t) 0x00, (register_pool + 21), false, false, true);
  //reg_defs[LSM9DS1_G_DATA_Z]           = DeviceRegister((BUS_ADDR + 0x1C), (uint16_t) 0x00, (register_pool + 23), false, false, true);
  //reg_defs[LSM9DS1_AG_CTRL_REG4]       = DeviceRegister((BUS_ADDR + 0x1E), (uint8_t)  0x38, (register_pool + 25), false, false, true);
  //reg_defs[LSM9DS1_A_CTRL_REG5]        = DeviceRegister((BUS_ADDR + 0x1F), (uint8_t)  0x38, (register_pool + 26), false, false, true);
  //reg_defs[LSM9DS1_A_CTRL_REG6]        = DeviceRegister((BUS_ADDR + 0x20), (uint8_t)  0x00, (register_pool + 27), false, false, true);
  //reg_defs[LSM9DS1_A_CTRL_REG7]        = DeviceRegister((BUS_ADDR + 0x21), (uint8_t)  0x00, (register_pool + 28), false, false, true);
  //reg_defs[LSM9DS1_AG_CTRL_REG8]       = DeviceRegister((BUS_ADDR + 0x22), (uint8_t)  0x04, (register_pool + 29), false, false, true);
  //reg_defs[LSM9DS1_AG_CTRL_REG9]       = DeviceRegister((BUS_ADDR + 0x23), (uint8_t)  0x00, (register_pool + 30), false, false, true);
  //reg_defs[LSM9DS1_AG_CTRL_REG10]      = DeviceRegister((BUS_ADDR + 0x24), (uint8_t)  0x00, (register_pool + 31), false, false, true);
  //reg_defs[LSM9DS1_A_INT_GEN_SRC]      = DeviceRegister((BUS_ADDR + 0x26), (uint8_t)  0x00, (register_pool + 32), false, false, true);
  //reg_defs[LSM9DS1_AG_STATUS_REG_ALT]  = DeviceRegister((BUS_ADDR + 0x27), (uint8_t)  0x00, (register_pool + 33), false, false, true);
  //reg_defs[LSM9DS1_A_DATA_X]           = DeviceRegister((BUS_ADDR + 0x28), (uint16_t) 0x00, (register_pool + 34), false, false, true);
  //reg_defs[LSM9DS1_A_DATA_Y]           = DeviceRegister((BUS_ADDR + 0x2A), (uint16_t) 0x00, (register_pool + 36), false, false, true);
  //reg_defs[LSM9DS1_A_DATA_Z]           = DeviceRegister((BUS_ADDR + 0x2C), (uint16_t) 0x00, (register_pool + 38), false, false, true);
  //reg_defs[LSM9DS1_AG_FIFO_CTRL]       = DeviceRegister((BUS_ADDR + 0x2E), (uint8_t)  0x00, (register_pool + 40), false, false, true);
  //reg_defs[LSM9DS1_AG_FIFO_SRC]        = DeviceRegister((BUS_ADDR + 0x2F), (uint8_t)  0x00, (register_pool + 41), false, false, true);
  //reg_defs[LSM9DS1_G_INT_GEN_CFG]      = DeviceRegister((BUS_ADDR + 0x30), (uint8_t)  0x00, (register_pool + 42), false, false, true);
  //reg_defs[LSM9DS1_G_INT_GEN_THS_X]    = DeviceRegister((BUS_ADDR + 0x31), (uint16_t) 0x00, (register_pool + 43), false, false, true);
  //reg_defs[LSM9DS1_G_INT_GEN_THS_Y]    = DeviceRegister((BUS_ADDR + 0x33), (uint16_t) 0x00, (register_pool + 45), false, false, true);
  //reg_defs[LSM9DS1_G_INT_GEN_THS_Z]    = DeviceRegister((BUS_ADDR + 0x35), (uint16_t) 0x00, (register_pool + 47), false, false, true);
  //reg_defs[LSM9DS1_G_INT_GEN_DURATION] = DeviceRegister((BUS_ADDR + 0x37), (uint8_t)  0x00, (register_pool + 49), false, false, true);

  // Preform our most commonly-used bus operations to minimize thrash and other kinds of overhead.
  /* Interrupt discovery. */
  preformed_busop_irq_mag.shouldReap(false);
  preformed_busop_irq_mag.devRegisterAdvance(false);
  preformed_busop_irq_mag.set_opcode(BusOpcode::RX);
  preformed_busop_irq_mag.callback = (BusOpCallback*) this;
  preformed_busop_irq_mag.buf      = regPtr(LSM9DS1_M_INT_SRC);
  preformed_busop_irq_mag.buf_len  = 1;
  preformed_busop_irq_mag.setParams(
    BUS_ADDR|0x80,
    preformed_busop_irq_mag.buf_len,
    1,
    (LSM9DS1_M_INT_SRC | 0xC0)
  );

  preformed_busop_read_mag.shouldReap(false);
  preformed_busop_read_mag.devRegisterAdvance(true);
  preformed_busop_read_mag.set_opcode(BusOpcode::RX);
  preformed_busop_read_mag.callback = (BusOpCallback*) this;
  preformed_busop_read_mag.buf      = regPtr(LSM9DS1_M_DATA_X);
  preformed_busop_read_mag.buf_len  = 6;
  preformed_busop_read_mag.setParams(
    BUS_ADDR|0x80,
    preformed_busop_read_mag.buf_len,
    1,
    (LSM9DS1_M_DATA_X | 0xC0)
  );

  // Preform our most commonly-used bus operations to minimize thrash and other kinds of overhead.
  preformed_busop_read_acc.shouldReap(false);
  preformed_busop_read_acc.devRegisterAdvance(true);
  preformed_busop_read_acc.set_opcode(BusOpcode::RX);
  preformed_busop_read_acc.callback = (BusOpCallback*) this;
  preformed_busop_read_acc.buf      = regPtr(LSM9DS1_A_DATA_X);
  preformed_busop_read_acc.buf_len  = 6;
  preformed_busop_read_acc.setParams(
    BUS_ADDR|0x80,
    preformed_busop_read_acc.buf_len,
    1,
    (LSM9DS1_A_DATA_X | 0x80)
  );

  preformed_busop_read_gyr.shouldReap(false);
  preformed_busop_read_gyr.devRegisterAdvance(true);
  preformed_busop_read_gyr.set_opcode(BusOpcode::RX);
  preformed_busop_read_gyr.callback = (BusOpCallback*) this;
  preformed_busop_read_gyr.buf      = regPtr(LSM9DS1_G_DATA_X);
  preformed_busop_read_gyr.buf_len  = 6;
  preformed_busop_read_gyr.setParams(
    BUS_ADDR|0x80,
    preformed_busop_read_gyr.buf_len,
    1,
    (LSM9DS1_G_DATA_X | 0x80)
  );

  preformed_busop_irq_0.shouldReap(false);
  preformed_busop_irq_0.devRegisterAdvance(true);
  preformed_busop_irq_0.set_opcode(BusOpcode::RX);
  preformed_busop_irq_0.callback = (BusOpCallback*) this;
  preformed_busop_irq_0.buf      = regPtr(LSM9DS1_G_INT_GEN_SRC);
  preformed_busop_irq_0.buf_len  = 22;
  preformed_busop_irq_0.setParams(
    BUS_ADDR|0x80,
    preformed_busop_irq_0.buf_len,
    1,
    (LSM9DS1_G_INT_GEN_SRC | 0x80)
  );

  preformed_busop_irq_1.shouldReap(false);
  preformed_busop_irq_1.devRegisterAdvance(true);
  preformed_busop_irq_1.set_opcode(BusOpcode::RX);
  preformed_busop_irq_1.callback = (BusOpCallback*) this;
  preformed_busop_irq_1.buf      = regPtr(LSM9DS1_A_INT_GEN_SRC);
  preformed_busop_irq_1.buf_len  = 3;
  preformed_busop_irq_1.setParams(
    BUS_ADDR|0x80,
    preformed_busop_irq_1.buf_len,
    1,
    (LSM9DS1_A_INT_GEN_SRC | 0x80)
  );

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
  IDX_T0 = LSM9DS1_M_OFFSET_X;
  IDX_T1 = LSM9DS1_M_OFFSET_Y;
  IDX_ID = LSM9DS1_M_WHO_AM_I;
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

  preformed_busop_irq_0.set_state(XferState::IDLE);
  preformed_busop_irq_1.set_state(XferState::IDLE);

  noise_floor_mag.set(0.0f, 0.0f, 0.0f);
  noise_floor_acc.set(0.0f, 0.0f, 0.0f);
  noise_floor_gyr.set(0.0f, 0.0f, 0.0f);

  // TODO: Blow away our idea of what is in the registers.
  // mark_it_zero();
  init();
  writeRegister(LSM9DS1_AG_CTRL_REG8, 0x01);
  writeRegister(LSM9DS1_M_CTRL_REG2, 0x04);
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
    // If there is more data on the way, we will let the callback do this for us.
    if (preformed_busop_read_acc.isIdle()) {
      readRegister((uint8_t) LSM9DS1_AG_FIFO_SRC);
    }
    if (preformed_busop_read_mag.isIdle()) {
      fire_preformed_bus_op(&preformed_busop_read_mag);
    }
  }

  if (preformed_busop_irq_1.isIdle()) {
    fire_preformed_bus_op(&preformed_busop_irq_1);
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


/*
*
*/
int8_t LSM9DS1::configure_sensor() {
  if (getVerbosity() > 3) Kernel::log("XM::configure_sensor()\n");

  writeRegister(LSM9DS1_M_CTRL_REG1,   (uint8_t*) &bulk_init_block_ag_0, 4);
  writeRegister(LSM9DS1_M_CTRL_REG5,   (uint8_t*) &bulk_init_block_ag_1, 6);
  writeRegister(LSM9DS1_G_INT_GEN_CFG, (uint8_t*) &bulk_init_block_ag_2, 8);
  writeRegister(LSM9DS1_M_CTRL_REG1,   (uint8_t*) &bulk_init_block_m, 5);

  // Latched active-high interrupt enabled on INT_M pin for all axes.
  writeRegister(LSM9DS1_M_INT_TSH, (uint8_t) 0x28);
  writeRegister(LSM9DS1_M_INT_CFG, (uint8_t) 0b11100101);

  // Continuous FIFO. Half-full (16) is threshold.
  writeRegister(LSM9DS1_AG_FIFO_CTRL, (uint8_t)  0b11010000);

  // If we are "init-pending", but not in readback phase, we need to dispatch the test writes.
  write_test_bytes();
  return 0;
}


/*
*
*/
bool LSM9DS1::is_setup_completed() {
  if (!present()) return false;

  if (!initComplete()) {   // TODO: Redundant.
    if (initPending()) {   // TODO: Redundant.
      //if (reg_defs[LSM9DS1_M_CTRL_REG1].dirty) return false;
      //if (reg_defs[LSM9DS1_M_CTRL_REG5].dirty) return false;
      //if (reg_defs[LSM9DS1_G_INT_GEN_CFG].dirty) return false;
      //if (reg_defs[LSM9DS1_AG_FIFO_CTRL].dirty) return false;
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
          integrator->deposit_log(&local_log);
        }
        // Rewrite valid values to those registers if necessary.
        //writeDirtyRegisters();
        _alter_flags(true, IMU_COMMON_FLAG_HW_WRITABLE);
        return true;
    }
    else {
      if (getVerbosity() > 2) {
        StringBuilder local_log;
        local_log.concatf("%s failed integrity check (index 0x%02x). Found 0x%02x. Expected 0x%02x.\n", imu_type(), IDX_T1, regValue(IDX_T1), io_test_val_1);
        integrator->deposit_log(&local_log);
      }
    }
  }
  else {
    if (getVerbosity() > 2) {
      StringBuilder local_log;
      local_log.concatf("%s failed integrity check (index 0x%02x). Found 0x%02x. Expected 0x%02x.\n", imu_type(), IDX_T0, regValue(IDX_T0), io_test_val_0);
      integrator->deposit_log(&local_log);
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
int8_t LSM9DS1::setDesiredState(State nu) {
  if (present() && (nu < IMUState::STAGE_1)) {
    // If we already know the sensor is there, why go back further than this?
    local_log.concatf("%s Trying to move to a state lower than allowed.\n", imu_type());
    Kernel::log(&local_log);
    return -1;
  }

  if (desired_state != nu) {
    if (!desired_state_attained()) {
      // TODO
      // The IMU is not at equilibrium. It may be ok to change the desired stage as long as we don't have
      //   bus operations pending.
      if (getVerbosity() > 2) {
        //local_log.concatf("%s tried to move to state %s while the IMU is off-balance (%s --> %s). Rejecting request.\n", imu_type(), getStateString(nu), getStateString(), getStateString(desired_state));
        local_log.concatf("%s tried to move to state %s while the IMU is off-balance (%s --> %s). We will allow this for now.\n", imu_type(), getStateString(nu), getStateString(), getStateString(desired_state));
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
    if (error_condition) {
      // We shouldn't be changing states if there is an error condition.
      // Reset is the only way to exit the condition at present.
      if (getVerbosity() > 2) {
        local_log.concatf("%s step_state() was called while we are in an error condition: %s\n", imu_type(), getErrorString());
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
        configure_sensor();
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
        integrator->state_change_notice(this, IMUState::STAGE_3, IMUState::STAGE_3);  // TODO: Wrong.
        sb_next_write = 0;
        readSensor();
        break;                                                       // TODO: Stop skipping calibrate().

      case IMUState::STAGE_4:  // Calibrated and idle.
        if (desiredState() == IMUState::STAGE_5) {
          // Enable the chained reads, and start the process rolling.
          readSensor();
        }
        else {
          // Downgrading to init state 3 (recalibrate).
          set_state(IMUState::STAGE_3);
          sb_next_write = 0;
          readSensor();
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
            readSensor();
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



int8_t LSM9DS1::writeRegister(uint8_t reg_index, uint8_t nu_val) {
  if (regExists(reg_index) && regWritable(reg_index)) {
    uint8_t* tmp = regPtr(reg_index);
    *tmp = nu_val;
    return writeRegister(reg_index, tmp, 1, false);
  }
  return IMUFault::REGISTER_UNDEFINED;
}
int8_t LSM9DS1::writeRegister(uint8_t reg_index, uint8_t *buf, uint8_t len) {     return writeRegister(reg_index, buf, len, (len > 1)); }
int8_t LSM9DS1::writeRegister(uint8_t reg_index, uint8_t *buf, uint8_t len, bool advance_regs) {
  if (!regWritable(reg_index)) {
    return IMUFault::NOT_WRITABLE;
  }
  else {
    uint8_t first_byte = reg_index;
    if (advance_regs) {
      // If we are advancing the register address,..
      // ..does the device even have that many...
      #ifdef __MANUVR_DEBUG
      StringBuilder _log;
      _log.concatf("SENSOR_ERROR_REG_NOT_DEFINED %d, LEN %d, idx = %d\n", BUS_ADDR, len, reg_index);
      Kernel::log(&_log);
      #endif
      if (regExists(reg_index + len)) {  // TODO: Sketchy.... might be unnecessary.
        return IMUFault::REGISTER_UNDEFINED;
      }
      // ...and is the entire range writable? Fail if not.
        for (uint8_t i = 0; i < len; i++) {
          if (regWritable(reg_index)) {
            #ifdef __MANUVR_DEBUG
              StringBuilder _log;
              _log.concatf("IMUFault::NOT_WRITABLE %d\n", BUS_ADDR);
              Kernel::log(&_log);
            #endif
            return IMUFault::NOT_WRITABLE;
          }
        }
        first_byte |= 0x40;
    }

    SPIBusOp* op = ((CPLDDriver*)cpld)->new_op();
    op->devRegisterAdvance(advance_regs);
    op->set_opcode(BusOpcode::TX);
    op->buf             = buf;
    op->buf_len         = len;
    op->callback        = (BusOpCallback*) this;
    op->setParams(BUS_ADDR, len, 1, first_byte);

    if (profile()) {
      op->profile(true);
    }
    ((CPLDDriver*)cpld)->queue_io_job(op);
  }
  return IMUFault::NO_ERROR;
}


/*
* Behavior:
* -----------------------------------------------------------------------
* reg_index     Device access will begin at the specified register index.
* buf           Pointer where read data will be deposited.
* len           How much data will we read?
* advance_regs  Are we reading x bytes from sequential registers?
*                 If false, will read the (reg_index) (len) times.
* (return)      IMU error code.
*
* Override rules:
* -----------------------------------------------------------------------
* 2 parameters   Length of one is assumed.
* 3 parameters   Lengths of more than 1 will be assumed to draw
*                  from sequential registers in the device.
*/
int8_t LSM9DS1::readRegister(uint8_t reg_index, uint8_t *buf, uint8_t len) {   return readRegister(reg_index, buf, len, (len > 1)); }
int8_t LSM9DS1::readRegister(uint8_t reg_index, uint8_t *buf, uint8_t len, bool advance_regs) {
  uint8_t first_byte = reg_index | 0x80;
  if (advance_regs) {
    // Mark all the registers covered by the range as being unread.
    int temp_len = len;
    int temp_idx = reg_index;
    while ((temp_len < len) && regExists(temp_idx)) {
      temp_len++;
      temp_idx++;
    }
    first_byte |= 0x40;
  }

  SPIBusOp* op = ((CPLDDriver*)cpld)->new_op();
  op->devRegisterAdvance(advance_regs);
  op->set_opcode(BusOpcode::RX);
  op->buf             = buf;
  op->buf_len         = len;
  op->callback        = (BusOpCallback*) this;
  op->setParams(BUS_ADDR|0x80, len, 1, first_byte);

  if (profile()) {
    op->profile(true);
  }
  ((CPLDDriver*)cpld)->queue_io_job(op);

  return IMUFault::NO_ERROR;
}


int8_t LSM9DS1::readRegister(uint8_t idx) {
  return readRegister(idx, regPtr(idx), 1, false);
}


/*
* Convenience fxn. Returns 0 if register index is out of bounds.
*/
unsigned int LSM9DS1::regValue(uint8_t idx) {
  switch (idx) {
    case 2:
    case 4:
      return 0;
    case 1:
    default:
      return 1;
  }
}


/*
*
*/
bool LSM9DS1::regWritable(uint8_t idx) {
  switch (idx) {
    case 2:
    case 4:
      return true;
    case 1:
    default:
      return false;
  }
}


/*
* Convenience fxn. Returns 0 if register index is out of bounds.
*/
bool LSM9DS1::regExists(uint8_t idx) {
  switch (idx) {
    case 2:
    case 4:
      return true;
    case 1:
    default:
      return false;
  }
}


/*
*
*/
uint8_t* LSM9DS1::regPtr(uint8_t idx) {
  switch (idx) {
    case 2:
    case 4:
      return &io_test_val_0;
    case 1:
    default:
      return &io_test_val_1;
  }
}


/**
* This is the means by which the class sends one of its pre-formed bus operations to the bus
*   manager.
*
* @param  A pointer to the pre-formed bus operation that the class wishes dispatched.
* @return true on success. False on failure.
*/
bool LSM9DS1::fire_preformed_bus_op(SPIBusOp* op) {
  if (reset_preformed_queue_item(op) ) {
    if (profile()) profiler_read_begin = micros();

    if (0 != ((CPLDDriver*)cpld)->queue_io_job(op)) {
      return false;
    }
  }
  return true;
}





/**
* We need to control for multiple-insertion conditions at this point. The passed-in
*   parameter is the bus operation that is about to be inserted into the bus queue.
* If we see that the bus op is not IDLE (IE, it is already waiting to be run, or is BEING run),
*   we return false without changing the operation. Otherwise, we reset it and give the "GO"
*   for insertion.
*
* @return true on success. False on failure.
*/
bool LSM9DS1::reset_preformed_queue_item(SPIBusOp* op) {
  switch (op->get_state()) {
    case XferState::IDLE:
      break;
    case XferState::INITIATE:
    case XferState::ADDR:
    case XferState::TX_WAIT:
    case XferState::RX_WAIT:
    case XferState::STOP:
    case XferState::COMPLETE:
    default:   // Functions like a primitive semaphore.
      if (getVerbosity() > 2) {
        local_log.concatf("reset_preformed_queue_item() failed because it had state %s\n", op->getStateString());
        op->printDebug(&local_log);
        Kernel::log(&local_log);
      }
      return false;
  }

  op->set_state(XferState::IDLE);
  return true;
}



/**
* NULL-checked upstream.
*/
void LSM9DS1::dumpPreformedElements(StringBuilder *output) {
  output->concat("--- Vector read (Mag)\n");
  preformed_busop_read_mag.printDebug(output);
  output->concat("--- Vector read (Accel)\n");
  preformed_busop_read_acc.printDebug(output);
  output->concat("--- Vector read (Gyr)\n");
  preformed_busop_read_gyr.printDebug(output);

  output->concat("--- IRQ mag\n");
  preformed_busop_irq_mag.printDebug(output);
  output->concat("--- IRQ pin1\n");
  preformed_busop_irq_0.printDebug(output);
  output->concat("--- IRQ pin2\n");
  preformed_busop_irq_1.printDebug(output);
  output->concat("\n");
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


/*******************************************************************************
* ___     _       _                      These members are mandatory overrides
*  |   / / \ o   | \  _     o  _  _      for implementing I/O callbacks. They
* _|_ /  \_/ o   |_/ (/_ \/ | (_ (/_     are also implemented by Adapters.
*******************************************************************************/

/**
* Called prior to the given bus operation beginning.
* Returning 0 will allow the operation to continue.
* Returning anything else will fail the operation with IO_RECALL.
*   Operations failed this way will have their callbacks invoked as normal.
*
* @param  _op  The bus operation that was completed.
* @return 0 to run the op, or non-zero to cancel it.
*/
int8_t LSM9DS1::io_op_callahead(BusOp* _op) {
  return 0;
}


/*
* Ultimately, all bus access this class does passes to this function as its last-stop
*   before becoming folded into the SPI bus queue.
*/
int8_t LSM9DS1::queue_io_job(BusOp* _op) {
  if (nullptr == _op) return -1;   // This should never happen.
  SPIBusOp* op = (SPIBusOp*) _op;
  op->callback = (BusOpCallback*) this;         // Notify us of the results.
  return ((CPLDDriver*)cpld)->queue_io_job(op);     // Pass it to the CPLD for bus access.
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
  uint8_t access_idx = op->getTransferParam(3);
  if (true) {  // TODO: Horrid. Wrong.
    io_op_callback_mag(op);
  }
  else {
    io_op_callback_ag(op);
  }

  if (local_log.length() > 0) Kernel::log(&local_log);
  return return_value;
}
