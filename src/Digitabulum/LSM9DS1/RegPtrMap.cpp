/*
File:   RegPtrMap.cpp
Author: J. Ian Lindsay
Date:   2017.02.03

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

/*
* The default values of the registers named in the enum class RegID.
* Some of these values should be construed as being 16-bit, but the size of
*   these arrays must remain the same.
*/
const uint8_t _imu_reg_defaults[] = {
  0x00, 0x00, 0x00,  // M: 16-bit offset registers
  0x3d, 0x40,
  0x00, 0x03, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00,  // M: 16-bit data registers
  0x08, 0x00,
  0x00,              // M: 16-bit threshold register
  // This is where the AG registers start.
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x68, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00,              // I: 16-bit temperature register (11-bit)
  0x00,
  0x00, 0x00, 0x00,  // G: 16-bit gyro data registers
  0x38, 0x38, 0x00, 0x00, 0x04, 0x00, 0x00,
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

/* The string representations of the registers named in the enum class RegID. */
const char* _imu_register_names[] = {
  "M_OFFSET_X", "M_OFFSET_Y", "M_OFFSET_Z",
  "M_WHO_AM_I", "M_CTRL_REG1",
  "M_CTRL_REG2", "M_CTRL_REG3", "M_CTRL_REG4", "M_CTRL_REG5", "M_STATUS_REG",
  "M_DATA_X", "M_DATA_Y", "M_DATA_Z",
  "M_INT_CFG", "M_INT_SRC",
  "M_INT_TSH",
  // This is where the AG registers start.
  "AG_ACT_THS", "AG_ACT_DUR", "A_INT_GEN_CFG", "A_INT_GEN_THS_X",
  "A_INT_GEN_THS_Y", "A_INT_GEN_THS_Z", "A_INT_GEN_DURATION", "G_REFERENCE",
  "AG_INT1_CTRL", "AG_INT2_CTRL", "AG_WHO_AM_I", "G_CTRL_REG1",
  "G_CTRL_REG2", "G_CTRL_REG3", "G_ORIENT_CFG", "G_INT_GEN_SRC",
  "AG_DATA_TEMP",
  "AG_STATUS_REG",
  "G_DATA_X", "G_DATA_Y", "G_DATA_Z",
  "AG_CTRL_REG4", "A_CTRL_REG5", "A_CTRL_REG6", "A_CTRL_REG7",
  "AG_CTRL_REG8", "AG_CTRL_REG9", "AG_CTRL_REG10",
  "A_INT_GEN_SRC", "AG_STATUS_REG_ALT",
  "A_DATA_X", "A_DATA_Y", "A_DATA_Z",
  "AG_FIFO_CTRL", "AG_FIFO_SRC", "G_INT_GEN_CFG",
  "G_INT_GEN_THS_X", "G_INT_GEN_THS_Y", "G_INT_GEN_THS_Z",
  "G_INT_GEN_DURATION"
};


// TODO: Incurring two branches might be cheaper than a const map. Investigate.
RegID RegPtrMap::regIdFromAddr(uint8_t dev_addr, uint8_t reg_addr) {
  if (dev_addr < CPLD_REG_IMU_DM_P_I) {  // Magnetic aspect.
    switch (reg_addr & 0x3F) {
      case 0x05: return RegID::M_OFFSET_X;
      case 0x07: return RegID::M_OFFSET_Y;
      case 0x09: return RegID::M_OFFSET_Z;
      case 0x0f: return RegID::M_WHO_AM_I;
      case 0x20: return RegID::M_CTRL_REG1;
      case 0x21: return RegID::M_CTRL_REG2;
      case 0x22: return RegID::M_CTRL_REG3;
      case 0x23: return RegID::M_CTRL_REG4;
      case 0x24: return RegID::M_CTRL_REG5;
      case 0x27: return RegID::M_STATUS_REG;
      case 0x28: return RegID::M_DATA_X;
      case 0x2a: return RegID::M_DATA_Y;
      case 0x2c: return RegID::M_DATA_Z;
      case 0x30: return RegID::M_INT_CFG;
      case 0x31: return RegID::M_INT_SRC;
      case 0x32: return RegID::M_INT_TSH;
    }
  }
  else {
    switch (reg_addr & 0x3F) {
      case 0x04: return RegID::AG_ACT_THS;
      case 0x05: return RegID::AG_ACT_DUR;
      case 0x06: return RegID::A_INT_GEN_CFG;
      case 0x07: return RegID::A_INT_GEN_THS_X;
      case 0x08: return RegID::A_INT_GEN_THS_Y;
      case 0x09: return RegID::A_INT_GEN_THS_Z;
      case 0x0a: return RegID::A_INT_GEN_DURATION;
      case 0x0b: return RegID::G_REFERENCE;
      case 0x0c: return RegID::AG_INT1_CTRL;
      case 0x0d: return RegID::AG_INT2_CTRL;
      case 0x0f: return RegID::AG_WHO_AM_I;
      case 0x10: return RegID::G_CTRL_REG1;
      case 0x11: return RegID::G_CTRL_REG2;
      case 0x12: return RegID::G_CTRL_REG3;
      case 0x13: return RegID::G_ORIENT_CFG;
      case 0x14: return RegID::G_INT_GEN_SRC;
      case 0x15: return RegID::AG_DATA_TEMP;
      case 0x17: return RegID::AG_STATUS_REG;
      case 0x18: return RegID::G_DATA_X;
      case 0x1a: return RegID::G_DATA_Y;
      case 0x1c: return RegID::G_DATA_Z;
      case 0x1e: return RegID::AG_CTRL_REG4;
      case 0x1f: return RegID::A_CTRL_REG5;
      case 0x20: return RegID::A_CTRL_REG6;
      case 0x21: return RegID::A_CTRL_REG7;
      case 0x22: return RegID::AG_CTRL_REG8;
      case 0x23: return RegID::AG_CTRL_REG9;
      case 0x24: return RegID::AG_CTRL_REG10;
      case 0x26: return RegID::A_INT_GEN_SRC;
      case 0x27: return RegID::AG_STATUS_REG_ALT;
      case 0x28: return RegID::A_DATA_X;
      case 0x2a: return RegID::A_DATA_Y;
      case 0x2c: return RegID::A_DATA_Z;
      case 0x2e: return RegID::AG_FIFO_CTRL;
      case 0x2f: return RegID::AG_FIFO_SRC;
      case 0x30: return RegID::G_INT_GEN_CFG;
      case 0x31: return RegID::G_INT_GEN_THS_X;
      case 0x33: return RegID::G_INT_GEN_THS_Y;
      case 0x35: return RegID::G_INT_GEN_THS_Z;
      case 0x37: return RegID::G_INT_GEN_DURATION;
    }
  }
}


/**
* Print the IMU register name.
*
* @return const char*
*/
const char* RegPtrMap::regNameString(RegID id) {
  return _imu_register_names[(uint8_t) id];
}

const uint8_t RegPtrMap::regAddr(RegID id) {
  return _imu_address_map[(uint8_t) id];
}

const uint8_t RegPtrMap::regWidth(RegID id) {
  return _imu_register_width_map[(uint8_t) id];
}

const bool RegPtrMap::regWritable(RegID id) {
  return _imu_register_writable_map[(uint8_t) id];
}


/*
* Looks at our local offset table to obtain value. This was set at instantiation
*   by ManaManager.
* TODO: This member is a horrible liability.... Not all of these registers will have a local pointer.
*/
const uint8_t* RegPtrMap::regPtr(RegID idx) const {
  switch (idx) {
    case RegID::M_OFFSET_X:          return ((uint8_t*) M_OFFSETS)+0;
    case RegID::M_OFFSET_Y:          return ((uint8_t*) M_OFFSETS)+2;
    case RegID::M_OFFSET_Z:          return ((uint8_t*) M_OFFSETS)+4;
    case RegID::M_WHO_AM_I:          break;
    case RegID::M_CTRL_REG1:         break;
    case RegID::M_CTRL_REG2:         return M_CTRL+0;
    case RegID::M_CTRL_REG3:         break;
    case RegID::M_CTRL_REG4:         break;
    case RegID::M_CTRL_REG5:         break;
    case RegID::M_STATUS_REG:        return M_STATUS+0;
    case RegID::M_DATA_X:            break;
    case RegID::M_DATA_Y:            break;
    case RegID::M_DATA_Z:            break;
    case RegID::M_INT_CFG:           break;
    case RegID::M_INT_SRC:           return M_INT_SRC+0;
    case RegID::M_INT_TSH:           return ((uint8_t*) M_INT_TSH)+0;
    case RegID::AG_ACT_THS:          return AG_ACT+0;
    case RegID::AG_ACT_DUR:          return AG_ACT+1;
    case RegID::A_INT_GEN_CFG:       return AG_BLOCK_0+0;
    case RegID::A_INT_GEN_THS_X:     return AG_BLOCK_0+1;
    case RegID::A_INT_GEN_THS_Y:     return AG_BLOCK_0+2;
    case RegID::A_INT_GEN_THS_Z:     return AG_BLOCK_0+3;
    case RegID::A_INT_GEN_DURATION:  return AG_BLOCK_0+4;
    case RegID::G_REFERENCE:         return AG_BLOCK_0+5;
    case RegID::AG_INT1_CTRL:        break;
    case RegID::AG_INT2_CTRL:        break;
    case RegID::AG_WHO_AM_I:         break;
    case RegID::G_CTRL_REG1:         return AG_CTRL1_3+0;
    case RegID::G_CTRL_REG2:         return AG_CTRL1_3+1;
    case RegID::G_CTRL_REG3:         return AG_CTRL1_3+2;
    case RegID::G_ORIENT_CFG:        break;
    case RegID::G_INT_GEN_SRC:       break;
    case RegID::AG_DATA_TEMP:        break;
    case RegID::AG_STATUS_REG:       break;
    case RegID::G_DATA_X:            break;
    case RegID::G_DATA_Y:            break;
    case RegID::G_DATA_Z:            break;
    case RegID::AG_CTRL_REG4:        break;
    case RegID::A_CTRL_REG5:         break;
    case RegID::A_CTRL_REG6:         return AG_CTRL6_7+0;
    case RegID::A_CTRL_REG7:         return AG_CTRL6_7+1;
    case RegID::AG_CTRL_REG8:        break;
    case RegID::AG_CTRL_REG9:        break;
    case RegID::AG_CTRL_REG10:       break;
    case RegID::A_INT_GEN_SRC:       break;
    case RegID::AG_STATUS_REG_ALT:   break;
    case RegID::A_DATA_X:            break;
    case RegID::A_DATA_Y:            break;
    case RegID::A_DATA_Z:            break;
    case RegID::AG_FIFO_CTRL:        break;
    case RegID::AG_FIFO_SRC:         return FIFO_LVLS;
    case RegID::G_INT_GEN_CFG:       break;
    case RegID::G_INT_GEN_THS_X:     return ((uint8_t*) G_INT_THS)+0;
    case RegID::G_INT_GEN_THS_Y:     return ((uint8_t*) G_INT_THS)+2;
    case RegID::G_INT_GEN_THS_Z:     return ((uint8_t*) G_INT_THS)+4;
    case RegID::G_INT_GEN_DURATION:  return G_INT_DUR+0;
  }
  return nullptr;
}
