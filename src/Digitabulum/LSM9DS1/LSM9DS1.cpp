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

#include "LSM9DSx.h"
#include "IIU.h"
#include "../CPLDDriver/CPLDDriver.h"

extern unsigned long micros(void);
extern volatile CPLDDriver* cpld;


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

State _state_indicies[] = {
  State::STAGE_0,   // Undiscovered. Maybe absent.
  State::STAGE_1,   // Discovered, but not init'd.
  State::STAGE_2,   // Discovered and initiallized, but unknown register values.
  State::STAGE_3,   // Fully initialized and sync'd. Un-calibrated.
  State::STAGE_4,   // Calibrated and idle.
  State::STAGE_5,   // Calibrated and reading.
  State::FAULT,     // Fault.
  State::UNDEF      // Not a state-machine value. A return code to simplifiy error-checks.
};

/**
* Return an enumerator given the state index.
*
* @return enum State
*/
State LSM9DSx_Common::getStateByIndex(uint8_t state_idx) {
  if (state_idx < sizeof(_state_indicies)) {
    return _state_indicies[state_idx];
  }
  return State::UNDEF;
}


/**
* Print a human-readable representation of the IMU fault condition.
*
* @return const char*
*/
const char* LSM9DSx_Common::getErrorString(uint8_t fault_code) {
  switch (fault_code) {
    case IMU_ERROR_NO_ERROR               :  return "NO_ERROR";
    case IMU_ERROR_WRONG_IDENTITY         :  return "WRONG_IDENTITY";
    case IMU_ERROR_INVALID_PARAM_ID       :  return "INVALID_PARAM_ID";
    case IMU_ERROR_NOT_CALIBRATED         :  return "NOT_CALIBRATED";
    case IMU_ERROR_NOT_WRITABLE           :  return "NOT_WRITABLE";
    case IMU_ERROR_DATA_EXHAUSTED         :  return "DATA_EXHAUSTED";
    case IMU_ERROR_NOT_INITIALIZED        :  return "NOT_INITIALIZED";
    case IMU_ERROR_BUS_INSERTION_FAILED   :  return "INSERTION_FAILED";
    case IMU_ERROR_BUS_OPERATION_FAILED_R :  return "OPERATION_FAILED_R";
    case IMU_ERROR_BUS_OPERATION_FAILED_W :  return "OPERATION_FAILED_W";
  }
  return "<UNKNOWN>";
}


/**
* Print a human-readable representation of the IMU state.
*
* @return const char*
*/
const char* LSM9DSx_Common::getStateString(State state) {
  switch (state) {
    case State::UNDEF:    return "UNDEF";
    case State::FAULT:    return "FAULT";
    case State::STAGE_0:  return "STAGE_0";
    case State::STAGE_1:  return "STAGE_1";
    case State::STAGE_2:  return "STAGE_2";
    case State::STAGE_3:  return "STAGE_3";
    case State::STAGE_4:  return "STAGE_4";
    case State::STAGE_5:  return "STAGE_5";
  }
  return "<UNKNOWN>";
}



/****************************************************************************************************
*   _____
*  / ____|
* | |     ___  _ __ ___  _ __ ___   ___  _ __
* | |    / _ \| '_ ` _ \| '_ ` _ \ / _ \| '_ \
* | |___| (_) | | | | | | | | | | | (_) | | | |
*  \_____\___/|_| |_| |_|_| |_| |_|\___/|_| |_|
*
* Low-level functions that the devices have in common...
* This amounts to mostly register access.
* Should probably avoid putting things here that directly pertain to sensor functionality.
****************************************************************************************************/
//TODO: I'll work back up to this once it isn't in the way.
//LSM9DSx_Common::LSM9DSx_Common(uint8_t addr, uint8_t ident_idx, uint8_t idx_test_0, uint8_t idx_test_1, IIU* _integrator) :
//  IDX_T0(idx_test_0), IDX_T1(idx_test_1), BUS_ADDR(addr), IDX_ID(ident_idx)
//{
//  integrator = _integrator;
//  init();
//}

LSM9DSx_Common::LSM9DSx_Common() {
}


/**
* Called to init the common boilerplate for this sensor.
*
* @return  IMU_ERROR_NO_ERROR or appropriate failure code.
*/
int8_t LSM9DSx_Common::init() {
  // Force our states back to reset.
  imu_state          = State::STAGE_0;
  desired_state      = State::STAGE_0;
  error_condition    = IMU_ERROR_NO_ERROR;

  _imu_flags = 1;
  sample_count       = 0;
  time_stamp_base    = 0;
  *pending_samples   = 0;
  return IMU_ERROR_NO_ERROR;
}


/**
* Calling this function will reset the common class elements to their
*   default states.
*/
void LSM9DSx_Common::reset() {
  // TODO: Blow away our idea of what is in the registers.
  // mark_it_zero();
  init();
}



int8_t LSM9DSx_Common::identity_check() {
  if (present()) return IMU_ERROR_NO_ERROR;
  return readRegister(IDX_ID);
}


/**
* Calling this will cause us to generate two random bytes and write them to two
*   separate registers chosen by the extending class. Those writes (if successful)
*   should cause an automatic re-read of those same registers to check that the bytes
*   were written.
*/
void LSM9DSx_Common::write_test_bytes() {
  io_test_val_0 = (uint8_t) randomInt() % 128;
  io_test_val_1 = (uint8_t) randomInt() % 128;

  writeRegister(IDX_T0, io_test_val_0);
  writeRegister(IDX_T1, io_test_val_1);
}


/**
* Our purpose here is to verify that our test value comes back on a read. This is a
*   bus-integrity test that must be passed prior to entering INIT-3.
*
* @return true if the test passes. False otherwise.
*/
bool LSM9DSx_Common::integrity_check() {
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

  error_condition = IMU_ERROR_NOT_WRITABLE;
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
int8_t LSM9DSx_Common::setDesiredState(State nu) {
  if (present() && (nu < State::STAGE_1)) {
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
  return IMU_ERROR_NO_ERROR;
}


/**
* Assumes that the operation prior set the state to whatever is current.
*
* @return true if the state is stable, and the integrator should be notified.
*/
bool LSM9DSx_Common::step_state() {
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
      case State::STAGE_0:  // We think the IIU might be physicaly absent.
        //reset(); ?
        identity_check();
        break;

      case State::STAGE_1:  // We are sure the IMU is present, but we haven't done anything with it.
        configure_sensor();
        break;

      case State::STAGE_2:  // Discovered and initiallized, but unknown register values.
        if (is_setup_completed()) {
          bulk_refresh();
        }
        else {
          set_state(State::STAGE_1);
          return true;
        }
        break;

      case State::STAGE_3:  // Fully initialized and sync'd. Un-calibrated.
        integrator->state_change_notice(this, State::STAGE_3, State::STAGE_3);  // TODO: Wrong.
        sb_next_write = 0;
        readSensor();
        break;                                                       // TODO: Stop skipping calibrate().

      case State::STAGE_4:  // Calibrated and idle.
        if (desiredState() == State::STAGE_5) {
          // Enable the chained reads, and start the process rolling.
          readSensor();
        }
        else {
          // Downgrading to init state 3 (recalibrate).
          set_state(State::STAGE_3);
          sb_next_write = 0;
          readSensor();
          return true;
        }
        break;

      case State::STAGE_5:  // Calibrated and reading.
        switch (desiredState()) {
          case State::STAGE_4:   // Stop reads.
            set_state(State::STAGE_4);
            return false;   /// Note the slight break from convention... Careful...

          case State::STAGE_3:  // Downgrading to init state 3 (recalibrate).
            set_state(State::STAGE_3);
            sb_next_write = 0;
            readSensor();
            return true;
          case State::STAGE_5:  // Keep reading.
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



/**
* Reads every register in the sensor with maximum bux efficiency.
*
* @return non-zero if there was a problem inserting the read job.
*/
int8_t LSM9DSx_Common::bulk_refresh() {
  return (fire_preformed_bus_op(&full_register_refresh) ? IMU_ERROR_NO_ERROR : IMU_ERROR_BUS_INSERTION_FAILED);
}


/*
* Ultimately, all bus access this class does passes to this function as its last-stop
*   before becoming folded into the SPI bus queue.
*/
int8_t LSM9DSx_Common::queue_io_job(BusOp* _op) {
  if (NULL == _op) return -1;   // This should never happen.
  SPIBusOp* op = (SPIBusOp*) _op;
  op->callback = (BusOpCallback*) this;         // Notify us of the results.
  return ((CPLDDriver*)cpld)->queue_io_job(op);     // Pass it to the CPLD for bus access.
}


int8_t LSM9DSx_Common::writeRegister(uint8_t reg_index, uint8_t nu_val) {
  if (regExists(reg_index) && regWritable(reg_index)) {
    uint8_t* tmp = regPtr(reg_index);
    *tmp = nu_val;
    return writeRegister(reg_index, tmp, 1, false);
  }
  return IMU_ERROR_REGISTER_UNDEFINED;
}
int8_t LSM9DSx_Common::writeRegister(uint8_t reg_index, uint8_t *buf, uint8_t len) {     return writeRegister(reg_index, buf, len, (len > 1)); }
int8_t LSM9DSx_Common::writeRegister(uint8_t reg_index, uint8_t *buf, uint8_t len, bool advance_regs) {
  if (!regWritable(reg_index)) {
    return IMU_ERROR_NOT_WRITABLE;
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
        return IMU_ERROR_REGISTER_UNDEFINED;
      }
      // ...and is the entire range writable? Fail if not.
        for (uint8_t i = 0; i < len; i++) {
          if (regWritable(reg_index)) {
            #ifdef __MANUVR_DEBUG
              StringBuilder _log;
              _log.concatf("IMU_ERROR_NOT_WRITABLE %d\n", BUS_ADDR);
              Kernel::log(&_log);
            #endif
            return IMU_ERROR_NOT_WRITABLE;
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
  return IMU_ERROR_NO_ERROR;
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
int8_t LSM9DSx_Common::readRegister(uint8_t reg_index, uint8_t *buf, uint8_t len) {   return readRegister(reg_index, buf, len, (len > 1)); }
int8_t LSM9DSx_Common::readRegister(uint8_t reg_index, uint8_t *buf, uint8_t len, bool advance_regs) {
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

  return IMU_ERROR_NO_ERROR;
}


int8_t LSM9DSx_Common::readRegister(uint8_t idx) {
  return readRegister(idx, regPtr(idx), 1, false);
}


/*
* Convenience fxn. Returns 0 if register index is out of bounds.
*/
unsigned int LSM9DSx_Common::regValue(uint8_t idx) {
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
bool LSM9DSx_Common::regWritable(uint8_t idx) {
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
bool LSM9DSx_Common::regExists(uint8_t idx) {
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
uint8_t* LSM9DSx_Common::regPtr(uint8_t idx) {
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
bool LSM9DSx_Common::fire_preformed_bus_op(SPIBusOp* op) {
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
bool LSM9DSx_Common::reset_preformed_queue_item(SPIBusOp* op) {
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
void LSM9DSx_Common::dumpPreformedElements(StringBuilder *output) {
  output->concat("--- Full refresh\n");
  full_register_refresh.printDebug(output);
  output->concat("\n");
}


/**
* Dump the contents of this device to the logger.
*
* @param   StringBuilder* The buffer into which this fxn should write its output.
*/
void LSM9DSx_Common::dumpDevRegs(StringBuilder *output) {
  output->concatf("\n-------------------------------------------------------\n--- IMU 0x%04x  %s ==>  %s \n-------------------------------------------------------\n", BUS_ADDR, getStateString(imu_state), (desired_state_attained() ? "STABLE" : getStateString(desired_state)));
  output->concatf("--- sample_count        %d\n--- pending_samples     %d\n\n", sample_count, *pending_samples);
  if (getVerbosity() > 1) {
    output->concatf("--- calibration smpls   %d\n", sb_next_write);
    output->concatf("--- Base filter param   %d\n", base_filter_param);
  }
  output->concatf("--- Error condition     %s\n---\n", getErrorString(error_condition));
}
