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

#include "LSM9DS1_M.h"
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
const UpdateRate2Hertz LSM9DS1_M::rate_settings_mag[MAXIMUM_RATE_INDEX_MAG] = {
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
* This is a generic table of scales versus unit-per-bit for 16-bit types.
* This needs to be updated with actual datasheet values.
*/
const GainErrorMap LSM9DS1_M::error_map_mag[MAXIMUM_GAIN_INDEX_MAG] = {
  {4,  (4/32768.0f),  0.000061},
  {8,  (8/32768.0f),  0.000122},
  {12, (12/32768.0f), 0.00032},
  {16, (16/32768.0f), 0.000244}
};


const float LSM9DS1_M::max_range_vect_mag   = 16.0;




/****************************************************************************************************
*  __  __                        _                       _
* |  \/  |                      | |                     | |
* | \  / | __ _  __ _ _ __   ___| |_ ___  _ __ ___   ___| |_ ___ _ __
* | |\/| |/ _` |/ _` | '_ \ / _ \ __/ _ \| '_ ` _ \ / _ \ __/ _ \ '__|
* | |  | | (_| | (_| | | | |  __/ || (_) | | | | | |  __/ ||  __/ |
* |_|  |_|\__,_|\__, |_| |_|\___|\__\___/|_| |_| |_|\___|\__\___|_|
*                __/ |
*               |___/
****************************************************************************************************/
/*
* Magnetometer data
* Tests all the required registers for freshness and builds a Vector with the new reading
*   if possible.
* Returns...
*   0 on success with no vector.
*   1 on success with new vector.
*  -1 on error.
*/
int8_t LSM9DS1_M::collect_reading_mag() {
  reg_defs[LSM9DS1_M_DATA_X].unread = false;
  reg_defs[LSM9DS1_M_DATA_Y].unread = false;
  reg_defs[LSM9DS1_M_DATA_Z].unread = false;

  /* Ok... so we know that if we got here, the pre-formed bus op is freshly execed,
       so we are going to grab its ending timestamp and populate the measurement's
       field with the value corrected for boot-time.
     TODO: This will have to revisited later to account for clock-wrap in a clean manner.
       Until then, it will cause us strange bugs, and possibly even a crash at the wrap.
  */

  Vector3<int16_t> reflection_vector_mag(LegendManager::reflection_mag.x, LegendManager::reflection_mag.y, LegendManager::reflection_mag.z);

  float scaler = error_map_mag[scale_mag].per_lsb;
  //float x = ((((int16_t)regValue(LSM9DS1_AG_DATA_X_M) - noise_floor_mag_mag.x) * reflection_vector_mag.x) * scaler);
  //float y = ((((int16_t)regValue(LSM9DS1_AG_DATA_Y_M) - noise_floor_mag_mag.y) * reflection_vector_mag.y) * scaler);
  //float z = ((((int16_t)regValue(LSM9DS1_AG_DATA_Z_M) - noise_floor_mag_mag.z) * reflection_vector_mag.z) * scaler);
  float x = ((((int16_t)regValue(LSM9DS1_M_DATA_X)) * reflection_vector_mag.x) * scaler);
  float y = ((((int16_t)regValue(LSM9DS1_M_DATA_Y)) * reflection_vector_mag.y) * scaler);
  float z = ((((int16_t)regValue(LSM9DS1_M_DATA_Z)) * reflection_vector_mag.z) * scaler);

  last_val_mag(x, y, z);
  integrator->pushMeasurement(IMU_FLAG_MAG_DATA, x, y, z, rate_settings_mag[update_rate_mag].ts_delta);

  return 1;
}



/**
* Call to rescale the sensor.
*/
int8_t LSM9DS1_M::request_rescale_mag(uint8_t nu_scale_idx) {
  if (nu_scale_idx < MAXIMUM_GAIN_INDEX_MAG) {
    if (scale_mag != nu_scale_idx) {
      if (verbosity > 2) Kernel::log("request_rescale_mag():\tRescaling magnetometer.\n");
      SPIDeviceWithRegisters::writeRegister(LSM9DS1_M_CTRL_REG2, (nu_scale_idx << 5));
    }
  }
  return 0;
}


/**
* Call to alter sample rate.
*/
int8_t LSM9DS1_M::set_sample_rate_mag(uint8_t nu_srate_idx) {
  if (nu_srate_idx < MAXIMUM_RATE_INDEX_MAG) {
    if (update_rate_mag != nu_srate_idx) {
      uint8_t temp8 = regValue(LSM9DS1_M_CTRL_REG1);
      if (0 == nu_srate_idx) {
        // Power the sensor down.
      }
      else {
        if (verbosity > 2) Kernel::log("set_sample_rate_mag():\tMagnetometer sample rate change.\n");
        temp8 =  (temp8 & ~0x1C) | ((nu_srate_idx-1) << 2);
      }
      update_rate_mag = nu_srate_idx;
      return SPIDeviceWithRegisters::writeRegister(LSM9DS1_M_CTRL_REG1, temp8);
    }
  }
  return 0;
}



/****************************************************************************************************
*   ___ _              ___      _ _              _      _
*  / __| |__ _ ______ | _ ) ___(_) |___ _ _ _ __| |__ _| |_ ___
* | (__| / _` (_-<_-< | _ \/ _ \ | / -_) '_| '_ \ / _` |  _/ -_)
*  \___|_\__,_/__/__/ |___/\___/_|_\___|_| | .__/_\__,_|\__\___|
*                                          |_|
* Constructors/destructors, class initialization functions and so-forth...
****************************************************************************************************/

LSM9DS1_M::LSM9DS1_M(uint8_t address, IIU* _integrator) : LSM9DSx_Common("GY ", address, _integrator, 23) {
  // First, we should define our registers....
  // 16 registers. 23 bytes
  register_pool = (uint8_t*) malloc(23);

  // Now we should give them initial definitions. This is our chance to set default configs.
  reg_defs[LSM9DS1_M_OFFSET_X]      = DeviceRegister((bus_addr + 0x05), (uint16_t) 0x00, (register_pool +  0), false, false, true);
  reg_defs[LSM9DS1_M_OFFSET_Y]      = DeviceRegister((bus_addr + 0x07), (uint16_t) 0x00, (register_pool +  2), false, false, true);
  reg_defs[LSM9DS1_M_OFFSET_Z]      = DeviceRegister((bus_addr + 0x09), (uint16_t) 0x00, (register_pool +  4), false, false, true);
  reg_defs[LSM9DS1_M_WHO_AM_I]      = DeviceRegister((bus_addr + 0x0F), (uint8_t)  0x00, (register_pool +  6), false, false, false);
  reg_defs[LSM9DS1_M_CTRL_REG1]     = DeviceRegister((bus_addr + 0x20), (uint8_t)  0x10, (register_pool +  7), false, false, true );  //
  reg_defs[LSM9DS1_M_CTRL_REG2]     = DeviceRegister((bus_addr + 0x21), (uint8_t)  0x00, (register_pool +  8), false, false, true );  //
  reg_defs[LSM9DS1_M_CTRL_REG3]     = DeviceRegister((bus_addr + 0x22), (uint8_t)  0x03, (register_pool +  9), false, false, true );  //
  reg_defs[LSM9DS1_M_CTRL_REG4]     = DeviceRegister((bus_addr + 0x23), (uint8_t)  0x00, (register_pool + 10), false, false, true );  //
  reg_defs[LSM9DS1_M_CTRL_REG5]     = DeviceRegister((bus_addr + 0x24), (uint8_t)  0x00, (register_pool + 11), false, false, true );  //
  reg_defs[LSM9DS1_M_STATUS_REG]    = DeviceRegister((bus_addr + 0x27), (uint8_t)  0x00, (register_pool + 12), false, false, true );  //
  reg_defs[LSM9DS1_M_DATA_X]        = DeviceRegister((bus_addr + 0x28), (uint16_t) 0x00, (register_pool + 13), false, false, false);
  reg_defs[LSM9DS1_M_DATA_Y]        = DeviceRegister((bus_addr + 0x2A), (uint16_t) 0x00, (register_pool + 15), false, false, false);
  reg_defs[LSM9DS1_M_DATA_Z]        = DeviceRegister((bus_addr + 0x2C), (uint16_t) 0x00, (register_pool + 17), false, false, false);
  reg_defs[LSM9DS1_M_INT_CFG]       = DeviceRegister((bus_addr + 0x30), (uint8_t)  0x08, (register_pool + 19), false, false, true);
  reg_defs[LSM9DS1_M_INT_SRC]       = DeviceRegister((bus_addr + 0x31), (uint8_t)  0x00, (register_pool + 20), false, false, false);
  reg_defs[LSM9DS1_M_INT_TSH]       = DeviceRegister((bus_addr + 0x32), (uint16_t) 0x00, (register_pool + 21), false, false, true);


  //reg_defs[LSM9DS1_M_CTRL_REG1]     = DeviceRegister((bus_addr + 0x20), (uint8_t)  0, (register_pool +  1), false, false, true );  // 760Hz update rate, 100 cutt-off, XYZ enabled, powered up.
  //reg_defs[LSM9DS1_M_CTRL_REG2]     = DeviceRegister((bus_addr + 0x21), (uint8_t)  0, (register_pool +  2), false, false, true );  // We want no filtering. Not sure if this is the correct way to achieve that...
  //reg_defs[LSM9DS1_M_CTRL_REG3]     = DeviceRegister((bus_addr + 0x22), (uint8_t)  0, (register_pool +  3), false, false, true );  // Open-drain interrupts on FIFO watermark. No boot status, active high.
  //reg_defs[LSM9DS1_M_CTRL_REG4]     = DeviceRegister((bus_addr + 0x23), (uint8_t)  0, (register_pool +  4), false, false, true );  // Block data update, LSB first, SPI 4-wire, no self-test, 245 degrees-per-sec.
  //reg_defs[LSM9DS1_M_CTRL_REG5]     = DeviceRegister((bus_addr + 0x24), (uint8_t)  0, (register_pool +  5), false, false, true );  // No factory data reset, Enable FIFO, disable high-pass filtering.
  //reg_defs[LSM9DS1_M_REFERENCE]     = DeviceRegister((bus_addr + 0x25), (uint8_t)  0, (register_pool +  6), false, false, true );  // Register that stores thresholds for angular rotation causing interrupts.
  //reg_defs[LSM9DS1_M_STATUS_REG]    = DeviceRegister((bus_addr + 0x27), (uint8_t)  0, (register_pool +  7), false, false, false);
  //reg_defs[LSM9DS1_M_FIFO_CTRL_REG] = DeviceRegister((bus_addr + 0x2E), (uint8_t)  0, (register_pool + 14), false, false, true );  // 0b00101000  FIFO mode. Watermark is 16 samples.
  //reg_defs[LSM9DS1_M_INT1_CFG]      = DeviceRegister((bus_addr + 0x30), (uint8_t)  0, (register_pool + 16), false, false, true );  // Threshold-related IRQ settings. Will probably want to use these for AGC.
  //reg_defs[LSM9DS1_M_INT1_DURATION] = DeviceRegister((bus_addr + 0x38), (uint8_t)  0, (register_pool + 24), false, false, true );  // Don't bother using threashold interrupts for the moment...

  /* Certain register indicies serve a common purpose in the base class. Define those... */
  idx_identity     = LSM9DS1_M_WHO_AM_I;

  idx_io_test_0    = LSM9DS1_M_OFFSET_X;
  idx_io_test_1    = LSM9DS1_M_OFFSET_Z;

  // Preform our most commonly-used bus operations to minimize thrash and other kinds of overhead.
  /* Interrupt discovery. */
  preformed_busop_irq_mag.shouldReap(false);
  preformed_busop_irq_mag.devRegisterAdvance(false);
  preformed_busop_irq_mag.callback = (SPIDeviceWithRegisters*) this;
  preformed_busop_irq_mag.opcode   = BusOpcode::RX;
  preformed_busop_irq_mag.reg_idx  = LSM9DS1_M_INT_SRC;
  preformed_busop_irq_mag.bus_addr = reg_defs[preformed_busop_irq_mag.reg_idx].addr | 0xC0;
  preformed_busop_irq_mag.buf      = reg_defs[preformed_busop_irq_mag.reg_idx].val;
  preformed_busop_irq_mag.buf_len  = 1;

  preformed_busop_read_mag.shouldReap(false);
  preformed_busop_read_mag.devRegisterAdvance(true);
  preformed_busop_read_mag.callback = (SPIDeviceWithRegisters*) this;
  preformed_busop_read_mag.opcode   = BusOpcode::RX;
  preformed_busop_read_mag.reg_idx  = LSM9DS1_M_DATA_X;
  preformed_busop_read_mag.bus_addr = reg_defs[preformed_busop_read_mag.reg_idx].addr | 0xC0;
  preformed_busop_read_mag.buf      = reg_defs[preformed_busop_read_mag.reg_idx].val;
  preformed_busop_read_mag.buf_len  = 6;

  last_val_mag(0.0f, 0.0f, 0.0f);
  noise_floor_mag(0, 0, 0);

  // Local class stuff...
  autoscale_mag       = false;
  scale_mag           = 0;
  update_rate_mag     = 0;
  discards_remain_mag = 0;
  discards_total_mag  = 0;

  // Superclass stuff...
  init();
}



LSM9DS1_M::~LSM9DS1_M(void) {
}



/****************************************************************************************************
* Members to be called from the integrator.                                                         *
****************************************************************************************************/
int8_t LSM9DS1_M::readSensor(void) {
  int8_t return_value = 0;
  if (!present()) {
    return -1;
  }
  if (!calibrated()) {
    //return IMU_ERROR_NOT_CALIBRATED;
  }

  if (initComplete()) {
    // If there is more data on the way, we will let the callback do this for us.
    if (preformed_busop_read_mag.isIdle()) {
      fire_preformed_bus_op(&preformed_busop_read_mag);
    }
  }
  return return_value;
}


/**
* Reads every non-FIFO register in the sensor with maximum bux efficiency.
* Note that this is an override. We make sure we can insert read operations
*   for our specific implementation before calling the upstream fxn to refresh
*   the basic things.
*
*/
int8_t LSM9DS1_M::bulk_refresh() {
  if (verbosity > 3) Kernel::log("LSM9DS1_M::bulk_refresh()\n");
  if (!present()) {
    return readRegister((uint8_t) LSM9DS1_M_WHO_AM_I);
  }

  if (fire_preformed_bus_op(&preformed_busop_irq_mag)) {
    return LSM9DSx_Common::bulk_refresh();
  }
  else {
    if (verbosity > 2) Kernel::log("\t Failed to fire preform irq_mag\n");
  }
  return IMU_ERROR_BUS_OPERATION_FAILED_R;
}



int8_t LSM9DS1_M::irq() {
  int8_t return_value = IMU_ERROR_NO_ERROR;

  if (verbosity > 3) Kernel::log("LSM9DS1_M::irq()\n");

  if (initComplete()) {
    if ( !fire_preformed_bus_op(&preformed_busop_irq_mag) ) {
      // Error-handling block.
    }
  }
  return return_value;
}



void LSM9DS1_M::reset() {
  autoscale_mag       = false;
  scale_mag           = 0;
  update_rate_mag     = 0;
  discards_remain_mag = 0;
  discards_total_mag  = 0;

  noise_floor_mag.set(0.0f, 0.0f, 0.0f);

  LSM9DSx_Common::reset();
  SPIDeviceWithRegisters::writeRegister(LSM9DS1_M_CTRL_REG2, 0x04);
}


uint8_t bulk_init_block_g[]   = { 0b01001111, 0b00000000, 0b00010000, 0b10010000, 0b01010000 };
uint8_t bulk_init_block_g_1[] = { 0b01101111, 0b01101111, 0b01101111};
uint8_t sample_rate_block_g[] = { 0b00000000, 0b00000000};


/****************************************************************************************************
* Runtime settings and status members.                                                              *
****************************************************************************************************/

// Init parameters for the magnetometer. Starting at CTRL_REG1.
uint8_t bulk_init_block_m[5] = {
  0b11011000,  // Temp compensated, mid-range X/Y performance, 40Hz output.
  0b00000000,  // 4 gauss,
  0b00000000,  // SPI read/write, continuous conversion. Normal power mode.
  0b00001000,  // Mid-range Z-axis performance, little-endian,
  0b00000000,  // Continuous (unblocked) data register update.
};


int8_t LSM9DS1_M::configure_sensor() {
  if (verbosity > 3) Kernel::log("LSM9DS1_M::configure_sensor()\n");

  writeRegister(LSM9DS1_M_CTRL_REG1, (uint8_t*) &bulk_init_block_m, 5);

  // Latched active-high interrupt enabled on INT_M pin for all axes.
  SPIDeviceWithRegisters::writeRegister(LSM9DS1_M_INT_TSH, (uint16_t) 0x2800, false);
  SPIDeviceWithRegisters::writeRegister(LSM9DS1_M_INT_CFG, (uint8_t)  0b11100101);

  write_test_bytes();
  return 0;
}


int8_t LSM9DS1_M::calibrate_from_data() {
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


/*
*
*/
bool LSM9DS1_M::is_setup_completed() {
  if (!present()) return false;

  if (!initComplete()) {
    if (initPending()) {
      if (reg_defs[LSM9DS1_M_INT_TSH].dirty) return false;
      if (reg_defs[LSM9DS1_M_INT_CFG].dirty) return false;
      if (reg_defs[LSM9DS1_M_CTRL_REG1].dirty) return false;
      if (reg_defs[LSM9DS1_M_CTRL_REG2].dirty) return false;
      if (reg_defs[LSM9DS1_M_CTRL_REG3].dirty) return false;
      if (reg_defs[LSM9DS1_M_CTRL_REG4].dirty) return false;
      if (reg_defs[LSM9DS1_M_CTRL_REG5].dirty) return false;
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
void LSM9DS1_M::dumpDevRegs(StringBuilder *output) {
  if (NULL == output) return;
  LSM9DSx_Common::dumpDevRegs(output);

  if (verbosity > 1) {
    output->concatf("--- update_rate_mag     %3.0f Hz\n", (double) rate_settings_mag[update_rate_mag].hertz);
  }
  if (verbosity > 2) {
    output->concatf("--- scale_mag           +/-%d deg/s\n", error_map_mag[scale_mag].scale);
    output->concatf("--- autoscale_mag       %s\n\n", (autoscale_mag ? "yes" : "no"));
    output->concatf("--- noise_floor_mag     (%d, %d, %d)\n", noise_floor_mag.x, noise_floor_mag.y, noise_floor_mag.z);
  }

  if (verbosity > 3) SPIDeviceWithRegisters::dumpDevRegs(output);
}


/*
* Dump the contents of this device to the logger.
* This is an override.
*/
void LSM9DS1_M::dumpPreformedElements(StringBuilder *output) {
  output->concat("--- Mag preformed elements\n");
  LSM9DSx_Common::dumpPreformedElements(output);

  output->concat("--- Vector read\n");
  preformed_busop_read_mag.printDebug(output);
  output->concat("--- IRQ pin\n");
  preformed_busop_irq_mag.printDebug(output);
  output->concat("\n");
}




/****************************************************************************************************
* Overrides from the SPI apparatus...                                                               *
****************************************************************************************************/

/*
* All notifications of bus activity enter the class here. This is probably where
*   we should act on data coming in.
*/
int8_t LSM9DS1_M::spi_op_callback(SPIBusOp* op) {
  int8_t return_value = SPI_CALLBACK_NOMINAL;

  if (-1 == op->reg_idx) {
    // Our class implementation considers this a serious problem.
    if (verbosity > 3) Kernel::log("~~~~~~~~LSM9DS1_M::spi_op_callback   (ERROR CASE -2)\n");
    return SPI_CALLBACK_ERROR;
  }

  // There is zero chance this object will be a null pointer unless it was done on purpose.
  if (op->hasError()) {
    if (verbosity > 3) {
      local_log.concat("~~~~~~~~LSM9DS1_M::spi_op_callback   (ERROR CASE -1)\n");
      op->printDebug(&local_log);
      Kernel::log(&local_log);
    }
    error_condition = (BusOpcode::RX == op->opcode) ? IMU_ERROR_BUS_OPERATION_FAILED_R : IMU_ERROR_BUS_OPERATION_FAILED_W;

    // TODO: Should think carefully, and...   return_value = SPI_CALLBACK_RECYCLE;   // Re-run the job.
    return SPI_CALLBACK_ERROR;
  }

  unsigned int access_len = op->buf_len;  // The access length lets us know how many things changed.
  unsigned int access_idx = op->reg_idx;  // The access length lets us know how many things changed.
  unsigned int value = regValue(access_idx);
  if (verbosity > 6) local_log.concatf("%s  G::spi_op_callback(0x%08x): value: %d \t access_idx  %d \t access_len: %d\n", op->getOpcodeString(), (uint32_t)((SPIOpCallback*) this), value, access_idx, access_len);

  /* Our first choice is: Did we just finish a WRITE or a READ? */
  /* READ Case-offs */
  if (BusOpcode::RX == op->opcode) {
    while ((access_len > 0) && (access_idx < reg_count)) {
      value = regValue(access_idx);
      access_len -= reg_defs[access_idx].len;   // Subtract the length.
      reg_defs[access_idx].unread = false;

      if (verbosity > 3) {
        local_log.concatf("\t GY R: access_idx  0x%02x   (0x%04x)\n", access_idx, (uint16_t) value);
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
        case LSM9DS1_M_WHO_AM_I:
          if (0x3D == value) {
            if (!present()) {
              set_state(State::STAGE_1);
              if (step_state()) {
                //integrator->init();   // Call this to kick the integrator into noticing our state change.
              }
            }
          }
          else {
            // We lost the IMU, perhaps...
            set_state(State::STAGE_0);
            error_condition = IMU_ERROR_WRONG_IDENTITY;
          }
          break;

        case LSM9DS1_M_DATA_X:
          collect_reading_mag();
        /* We don't address these registers byte-wise. Empty case for documentation's sake. */
        case LSM9DS1_M_DATA_Y:  break;
        case LSM9DS1_M_DATA_Z:  break;


        case LSM9DS1_M_CTRL_REG1:
          if (((value >> 2) & 0x07) < MAXIMUM_RATE_INDEX_MAG)  update_rate_mag = ((value >> 6) & 0x03)+1;
          break;

        case LSM9DS1_M_CTRL_REG2:
          if (((value >> 4) & 0x03) < MAXIMUM_GAIN_INDEX_MAG)  scale_mag = (value >> 5) & 0x03;
          break;

        case LSM9DS1_M_CTRL_REG3:
          power_to_mag = (value & 0x02);
          break;

        case LSM9DS1_M_CTRL_REG4:
          break;

        case LSM9DS1_M_CTRL_REG5:
          break;

        default:
          break;
      }
      if (op->devRegisterAdvance()) access_idx++;
    }

    if (profile) {
      profiler_read_end = micros();
      if (verbosity > 5) {
        local_log.concatf("\t G Operation took %uus\n", (unsigned long) profiler_read_end);
      }
    }
  }


  /* WRITE Case-offs */
  else if (BusOpcode::TX == op->opcode) {
    while ((access_len > 0) && (access_idx < reg_count)) {
      value = regValue(access_idx);
      access_len -= reg_defs[access_idx].len;   // Subtract the length.
      if (verbosity > 3) {
        local_log.concatf("\t G  W: access_idx  0x%02x   (0x%04x)\n", access_idx, (uint16_t) value);
      }
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
        case LSM9DS1_M_CTRL_REG1:
          if (((value >> 2) & 0x07) < MAXIMUM_RATE_INDEX_MAG)  update_rate_mag = ((value >> 6) & 0x03)+1;
          break;

        case LSM9DS1_M_CTRL_REG2:
          if (((value >> 4) & 0x03) < MAXIMUM_GAIN_INDEX_MAG)  scale_mag = (value >> 5) & 0x03;
          if (value & 0x04) { // Did we write here to reset?
            if (!present()) {
              integrator->init();
            }
          }
          break;

        case LSM9DS1_M_CTRL_REG3:
          power_to_mag = (value & 0x02);
          break;

        case LSM9DS1_M_CTRL_REG5:
          break;

        default:
          if (verbosity > 5) local_log.concatf("\t G Wrote an unimplemented register.\n");
          break;
      }
      if (op->devRegisterAdvance()) access_idx++;
    }
  }

  if (local_log.length() > 0) Kernel::log(&local_log);
  return return_value;
}
