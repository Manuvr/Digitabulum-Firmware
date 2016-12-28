/*
File:   LSM9DSx.h
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



================================================================================


Digitabulum refactor log: (Notes from r0, preserved until not-needed)
=============================================================================================
The register mechanism found new life in the i2c code, but we need to bring some of those
  advancements back into the hw driver classes. I don't think that direct re-use will be
  feasible, due to the weird nature of the "register protocol" in the ST parts. Probably not
  worth the extra complexity. Maybe later...
IMUs need to be aware of their own bus addresses so that bus access can be encapsulated more
  elegantly (versus now where we pass upstream to the CPLD). Maybe try to use EventManager
  for passing bus operations? Might be too much overhead.
  ---J. Ian Lindsay   Fri Nov 07 03:46:52 MST 2014
*/



#ifndef __LSM9DS1_COMMON_H
#define __LSM9DS1_COMMON_H

#include <DataStructures/InertialMeasurement.h>
#include <Platform/Peripherals/SPI/SPIBusOp.h>

class CPLDDriver;

/*
* There are three mutually-exclusive ways to talk to this chip. The choice
* can only be made by hardware. So this class needs to be initialized with
* the choice appropriate to your hardware configuration.
* If the device is to be used via i2c, there are 4 possible address permutations
* that must be provided after construction.
*/
#define LSM9DS1_BUS_MODES_I2C        0x00
#define LSM9DS1_BUS_MODES_SPI_4_WIRE 0x01
#define LSM9DS1_BUS_MODES_SPI_3_WIRE 0x02


/*
* These are possible error states for the IMU state-machine.
*/
#define IMU_ERROR_NO_ERROR                  0
#define IMU_ERROR_WRONG_IDENTITY            1
#define IMU_ERROR_INVALID_PARAM_ID          2
#define IMU_ERROR_NOT_CALIBRATED            3
#define IMU_ERROR_NOT_WRITABLE              4
#define IMU_ERROR_DATA_EXHAUSTED            5
#define IMU_ERROR_NOT_INITIALIZED           6
#define IMU_ERROR_BUS_INSERTION_FAILED      7
#define IMU_ERROR_BUS_OPERATION_FAILED_R    8
#define IMU_ERROR_BUS_OPERATION_FAILED_W    9
#define IMU_ERROR_REGISTER_UNDEFINED       10


#define IMU_COMMON_FLAG_VERBOSITY_MASK 0x0007
#define IMU_COMMON_FLAG_HW_WRITABLE    0x0008
#define IMU_COMMON_FLAG_PROFILING      0x0010
#define IMU_COMMON_FLAG_CANCEL_ERROR   0x0020
#define IMU_COMMON_FLAG_AUTOSCALE_0    0x0040
#define IMU_COMMON_FLAG_AUTOSCALE_1    0x0080
#define IMU_COMMON_FLAG_MAG_POWERED    0x2000
#define IMU_COMMON_FLAG_GYR_POWERED    0x4000
#define IMU_COMMON_FLAG_ACC_POWERED    0x8000


/*
* In addition to the natural ADC error, the sensor accuracy worsens at these rates for
* every degree C deviant of 25C...
*/
const float ACC_TEMPERATURE_DERATE = 0.015f;
const float MAG_TEMPERATURE_DERATE = 0.03f;
const float GYR_TEMPERATURE_DERATE = 0.02f;


/* We use this struct to map between update rates and timestamp deltas. */
typedef struct {
  float hertz;      // Frequency
  float ts_delta;   // Period (in seconds)
} UpdateRate2Hertz;


/* We use the struct to map between scales and error-rates. */
typedef struct {
  uint16_t scale;     // This is the maximum magnatude of the sensor reading at the given gain.
  float    per_lsb;   // Each LSB counts for this many of whatever unit.
  float    error;     // Given in the sensor's native unit. Each reading at this scale has this error.
} GainErrorMap;



class LSM9DSx_Common;   // Forward declaration
class IIU;              // Forward declaration of the IIU class.


/*
* These are the sensor states.
*/
enum class State {
  STAGE_0 = 0,  // Undiscovered. Maybe absent.
  STAGE_1,      // Discovered, but not init'd.
  STAGE_2,      // Discovered and initiallized, but unknown register values.
  STAGE_3,      // Fully initialized and sync'd. Un-calibrated.
  STAGE_4,      // Calibrated and idle.
  STAGE_5,      // Calibrated and reading.
  FAULT,        // Fault.
  UNDEF         // Not a state-machine value. A return code to simplifiy error-checks.
};



/*******************************************************************************
* Common members of the class...                                               *
*******************************************************************************/

/**
* The hardware driver for the LSM9DSx.
* This class is purely for abstraction, and is never instatntiated. It is only
*   intended to hold functions and members common to a single device package.
*/
class LSM9DSx_Common : public BusOpCallback {
  public:
    /* State-check functions. Inlined where practical. */
    inline State getState() {               return imu_state;                             }
    inline State desiredState() {           return desired_state;                         }

    inline bool present() {                 return (State::STAGE_0 != getState());        }
    inline bool initPending() {             return ((State::STAGE_1 == getState()) || (State::STAGE_2 == getState()));  }
    inline bool initReadback() {            return (State::STAGE_2 == getState());        }
    inline bool initComplete() {            return (State::STAGE_3 <= getState());        }
    inline bool calibrated() {              return (State::STAGE_4 <= getState());        }
    inline bool idle() {                    return (State::STAGE_4 == getState());        }
    inline bool reading() {                 return (State::STAGE_5 == getState());        }
    inline bool desired_state_attained() {  return (getState() == desiredState());        }

    inline bool profile() {         return _check_flags(IMU_COMMON_FLAG_PROFILING);     };
    inline bool cancel_error() {    return _check_flags(IMU_COMMON_FLAG_CANCEL_ERROR);  };
    inline void profile(bool x) {       _alter_flags(x, IMU_COMMON_FLAG_PROFILING);     };
    inline void cancel_error(bool x) {  _alter_flags(x, IMU_COMMON_FLAG_CANCEL_ERROR);  };

    int8_t setDesiredState(State);   // Used to set the state the OS wants the IMU class to acheive.
    void   write_test_bytes();
    bool   step_state();      // Used internally to move between states. TODO: Should be private.

    int8_t  init(void);
    virtual void reset(void);           // Reset our state without causing a re-init.

    virtual int8_t bulk_refresh();    // Read all the non-identity/non-FIFO registers in the device.

    /* Debug stuff... */
    virtual void dumpDevRegs(StringBuilder*);
    virtual void dumpPreformedElements(StringBuilder*);

    /* Overrides from the BusOpCallback interface */
    virtual int8_t io_op_callback(BusOp*) = 0;
    int8_t queue_io_job(BusOp*);         // Implemented here.

    /* Functions called by the IIU */
    virtual int8_t readSensor(void) =0;      // Call to poll the sensor's registers and take any appropriate action.
    //virtual int8_t enable(bool);             // Pass a boolean to turn the sensor on or off.
    //virtual int8_t set_base_filter_param(uint8_t nu_bw_idx) =0;

    /* Inlines for the specialized flag duty of get/set class verbosity. */
    inline uint8_t getVerbosity() {
      return (_imu_flags & IMU_COMMON_FLAG_VERBOSITY_MASK);
    };
    inline void  setVerbosity(uint8_t nu) {
      _imu_flags = (nu & IMU_COMMON_FLAG_VERBOSITY_MASK) | (_imu_flags & ~IMU_COMMON_FLAG_VERBOSITY_MASK);
    };

    inline const char* getStateString() {    return getStateString(imu_state);        }
    inline const char* getErrorString() {    return getErrorString(error_condition);  }

    static State getStateByIndex(uint8_t state_idx);
    static const char* getStateString(State state);
    static const char* getErrorString(uint8_t fault_code);



  protected:
    IIU* integrator = NULL;

    uint8_t BUS_ADDR;  // What is our address on the bus? TODO: const
    // TODO: r1 simplified things a great deal. All these members can probably DIAF.
    uint8_t IDX_T0; // TODO: Ought to be const if here at all.
    uint8_t IDX_T1; // TODO: Ought to be const if here at all.
    uint8_t IDX_ID; // TODO: Ought to be const if here at all.

    uint32_t  profiler_read_begin = 0;    // Profiling member.
    uint32_t  profiler_read_end   = 0;    // Profiling member.
    uint32_t  time_stamp_base  = 0;       // What time was it when we first started taking samples?
    uint32_t  last_sample_time = 0;       // What time was it when we first started taking samples?
    uint32_t  sample_count     = 0;       // How many samples have we read since init?
    uint8_t*  pending_samples  = nullptr; // How many samples are we expecting to arrive?

    uint16_t _imu_flags        = 1;     // Default verbosity of 1.

    uint8_t   io_test_val_0    = 0;     //
    uint8_t   io_test_val_1    = 0;     //

    int8_t    base_filter_param = 0;

    uint8_t   sb_next_read     = 0;
    uint8_t   sb_next_write    = 0;
    uint8_t   error_condition  = 0;

    State     imu_state        = State::STAGE_0;
    State     desired_state    = State::STAGE_0;

    SPIBusOp full_register_refresh;

    StringBuilder local_log;


    LSM9DSx_Common();

    /* These are higher-level fxns that are used as "macros" for specific patterns of */
    /*   register access. Common large-scale operations should go here.               */
    void   reset(uint8_t reg_idx);   // Reset our state without causing a re-init.

    int8_t writeRegister(uint8_t base_index, uint8_t nu_val);
    int8_t writeRegister(uint8_t base_index, uint8_t *buf, uint8_t len);
    int8_t writeRegister(uint8_t base_index, uint8_t *buf, uint8_t len, bool advance_regs);
    int8_t readRegister(uint8_t base_index, uint8_t *buf, uint8_t len);  // Overrides above fxn with advance_regs = false
    int8_t readRegister(uint8_t base_index, uint8_t *buf, uint8_t len, bool advance_regs);
    int8_t readRegister(uint8_t index);
    unsigned int regValue(uint8_t idx);
    bool regWritable(uint8_t idx);
    bool regExists(uint8_t idx);
    uint8_t* regPtr(uint8_t idx);
    /* This is the end of the low-level functions.                                    */

    /**
    * Sets the current IMU state without blowing away the high bits in the state member.
    */
    inline void set_state(State nu) {     imu_state = nu;   }

    /* Inlines for altering and reading the flags. */
    inline void _alter_flags(bool en, uint16_t mask) {
      _imu_flags = (en) ? (_imu_flags | mask) : (_imu_flags & ~mask);
    };
    inline bool _check_flags(uint16_t mask) {
      return (mask == (_imu_flags & mask));
    };


    int8_t identity_check();
    bool fire_preformed_bus_op(SPIBusOp* op);
    bool integrity_check();

    virtual bool is_setup_completed() =0;
    virtual int8_t configure_sensor() =0;
    virtual const char* imu_type()    =0;



  private:
    bool reset_preformed_queue_item(SPIBusOp* op);

};

#endif // __LSM9DS1_COMMON_H
