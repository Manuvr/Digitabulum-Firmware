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



#ifndef __LSM9DS1_MERGED_H__
#define __LSM9DS1_MERGED_H__

#include <DataStructures/InertialMeasurement.h>
#include <Platform/Peripherals/SPI/SPIBusOp.h>

class CPLDDriver;

/*
* These are possible error states for the IMU state-machine.
*/
enum class IMUFault {
  NO_ERROR               =  0,
  WRONG_IDENTITY         =  1,
  INVALID_PARAM_ID       =  2,
  NOT_CALIBRATED         =  3,
  NOT_WRITABLE           =  4,  // TODO: Cut. Should no longer be possible.
  DATA_EXHAUSTED         =  5,
  NOT_INITIALIZED        =  6,
  BUS_INSERTION_FAILED   =  7,  // TODO: Cut. Should no longer be possible.
  BUS_OPERATION_FAILED_R =  8,  // TODO: Cut. Should no longer be possible.
  BUS_OPERATION_FAILED_W =  9,  // TODO: Cut. Should no longer be possible.
  REGISTER_UNDEFINED     = 10   // TODO: Cut. Should no longer be possible.
};

/*
* These are the sensor states.
*/
enum class IMUState {
  STAGE_0 = 0,  // Undiscovered. Maybe absent.
  STAGE_1,      // Discovered, but not init'd.
  STAGE_2,      // Discovered and initiallized, but unknown register values.
  STAGE_3,      // Fully initialized and sync'd. Un-calibrated.
  STAGE_4,      // Calibrated and idle.
  STAGE_5,      // Calibrated and reading.
  FAULT,        // Fault.
  UNDEF         // Not a state-machine value. A return code to simplifiy error-checks.
};


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
const float MAG_TEMPERATURE_DERATE = 0.03f;
const float ACC_TEMPERATURE_DERATE = 0.015f;
const float GYR_TEMPERATURE_DERATE = 0.02f;


/* We use this struct to map between update rates and timestamp deltas. */
// TODO: Ne3ds mo4r const.
typedef struct {
  float hertz;      // Frequency
  float ts_delta;   // Period (in seconds)
} UpdateRate2Hertz;


/* We use the struct to map between scales and error-rates. */
// TODO: Ne3ds mo4r const.
typedef struct {
  uint16_t scale;     // This is the maximum magnatude of the sensor reading at the given gain.
  float    per_lsb;   // Each LSB counts for this many of whatever unit.
  float    error;     // Given in the sensor's native unit. Each reading at this scale has this error.
} GainErrorMap;


/*
* Internally-used register IDs. We need this abstraction because the addresses
*   within a given package are not disjoint sets.
* The order corrosponds to the order of the named register's occurance in the device.
*
* NOTE: These values are NOT register addresses, they are indicies. The given
*   order, and the fact that the first valid register starts at 0 is an
*   assumption made throughout this class.
*   Magnetometer registers occur first because the CPLD is organized that way.
* NOTE: In order to get away with using the restricted list, the sensors MUST be
*   configured to allow multiple sequential register access. The means for doing
*   this vary between mag/ag.
*/
enum class RegID {
  M_OFFSET_X = 0x00,  // 16-bit offset registers
  M_OFFSET_Y,         // 16-bit offset registers
  M_OFFSET_Z,         // 16-bit offset registers
  M_WHO_AM_I,
  M_CTRL_REG1,
  M_CTRL_REG2,
  M_CTRL_REG3,
  M_CTRL_REG4,
  M_CTRL_REG5,
  M_STATUS_REG,
  M_DATA_X,           // 16-bit data registers
  M_DATA_Y,           // 16-bit data registers
  M_DATA_Z,           // 16-bit data registers
  M_INT_CFG,
  M_INT_SRC,
  M_INT_TSH,          // 16-bit threshold register
  AG_ACT_THS,
  AG_ACT_DUR,
  A_INT_GEN_CFG,
  A_INT_GEN_THS_X,    // 8-bit threshold registers
  A_INT_GEN_THS_Y,    // 8-bit threshold registers
  A_INT_GEN_THS_Z,    // 8-bit threshold registers
  A_INT_GEN_DURATION,
  G_REFERENCE,
  AG_INT1_CTRL,
  AG_INT2_CTRL,
  AG_WHO_AM_I,
  G_CTRL_REG1,
  G_CTRL_REG2,
  G_CTRL_REG3,
  G_ORIENT_CFG,
  G_INT_GEN_SRC,
  AG_DATA_TEMP,       // 16-bit temperature register (11-bit)
  AG_STATUS_REG,
  G_DATA_X,           // 16-bit gyro data registers
  G_DATA_Y,           // 16-bit gyro data registers
  G_DATA_Z,           // 16-bit gyro data registers
  AG_CTRL_REG4,
  A_CTRL_REG5,
  A_CTRL_REG6,
  A_CTRL_REG7,
  AG_CTRL_REG8,
  AG_CTRL_REG9,
  AG_CTRL_REG10,
  A_INT_GEN_SRC,
  AG_STATUS_REG_ALT,
  A_DATA_X,           // 16-bit accelerometer data registers
  A_DATA_Y,           // 16-bit accelerometer data registers
  A_DATA_Z,           // 16-bit accelerometer data registers
  AG_FIFO_CTRL,
  AG_FIFO_SRC,
  G_INT_GEN_CFG,
  G_INT_GEN_THS_X,    // 16-bit threshold registers
  G_INT_GEN_THS_Y,    // 16-bit threshold registers
  G_INT_GEN_THS_Z,    // 16-bit threshold registers
  G_INT_GEN_DURATION
};




/*******************************************************************************
* This is the const object that is passed into the LSM9DS1 constructor. Its
*   values are pointers to the memory that represents the LSM9DS1 registers.
*******************************************************************************/
class RegPtrMap {
  // Because the data frames come in so fast, we need to double buffer them.
  //M_OFFSET_X = 0x00,  // 16-bit offset registers
  //M_OFFSET_Y,         // 16-bit offset registers
  //M_OFFSET_Z,         // 16-bit offset registers
  //M_CTRL_REG1,
  //M_CTRL_REG2,
  //M_CTRL_REG3,
  //M_CTRL_REG4,
  //M_CTRL_REG5,
  //M_STATUS_REG,
  //M_DATA_X,           // 16-bit data registers
  //M_DATA_Y,           // 16-bit data registers
  //M_DATA_Z,           // 16-bit data registers
  //M_INT_CFG,
  //M_INT_SRC,
  //M_INT_TSH,          // 16-bit threshold register

  //AG_ACT_THS,
  //AG_ACT_DUR,
  //A_INT_GEN_CFG,
  //A_INT_GEN_THS_X,    // 8-bit threshold registers
  //A_INT_GEN_THS_Y,    // 8-bit threshold registers
  //A_INT_GEN_THS_Z,    // 8-bit threshold registers
  //A_INT_GEN_DURATION,
  //G_REFERENCE,
  //AG_INT1_CTRL,
  //AG_INT2_CTRL,
  //AG_WHO_AM_I,
  //G_CTRL_REG1,
  //G_CTRL_REG2,
  //G_CTRL_REG3,
  //G_ORIENT_CFG,
  //G_INT_GEN_SRC,
  //AG_STATUS_REG,
  //AG_CTRL_REG4,
  //A_CTRL_REG5,
  //A_CTRL_REG6,
  //A_CTRL_REG7,
  //AG_CTRL_REG8,
  //AG_CTRL_REG9,
  //AG_CTRL_REG10,
  //A_INT_GEN_SRC,
  //AG_STATUS_REG_ALT,
  //AG_FIFO_CTRL,
  //AG_FIFO_SRC,
  //G_INT_GEN_CFG,
  //G_INT_GEN_THS_X,    // 16-bit threshold registers
  //G_INT_GEN_THS_Y,    // 16-bit threshold registers
  //G_INT_GEN_THS_Z,    // 16-bit threshold registers
  //G_INT_GEN_DURATION

  public:
    RegPtrMap(
      const uint8_t idx,
      const uint8_t* ag_activity,
      const uint8_t* ag_ctrl1_3,
      const uint8_t* ag_ctrl6_7,
      const uint8_t* ag_status,
      const uint8_t* fifo_src
    ) :
      AG_ACT((idx * 2) + ag_activity),
      AG_CTRL1_3((idx * 3) + ag_ctrl1_3),
      AG_CTRL6_7((idx * 2) + ag_ctrl6_7),
      AG_STATUS(idx + ag_status),
      FIFO_LVLS(idx + fifo_src)
    {};

    const uint8_t* regPtr(RegID) const;

    static RegID regIdFromAddr(uint8_t);


  private:
    // Some registers are not included in this list if their function can be
    //   handled entirely within ManuManager.
    const uint8_t* AG_ACT;  // AG_ACT_THS, AG_ACT_DUR
    const uint8_t* AG_BLOCK_0;  // A_INT_GEN_CFG, A_INT_GEN_THS_X, A_INT_GEN_THS_Y, A_INT_GEN_THS_Z, A_INT_GEN_DURATION, G_REFERENCE,
    const uint8_t* AG_CTRL1_3;  // G_CTRL_REG1, G_CTRL_REG2, G_CTRL_REG3
    const uint8_t* AG_CTRL6_7;  // A_CTRL_REG6, A_CTRL_REG7
    const uint8_t* AG_STATUS;   // AG_STATUS_REG
    const uint8_t* FIFO_LVLS;   // AG_FIFO_SRC
};


/*******************************************************************************
* The LSM9DS1 class. Acc, Gyr, and Mag are all handled here, and the work of
*   distinguishing between them falls on the CPLD hardware and ManuManager.
*******************************************************************************/
class LSM9DS1 {
  public:
    LSM9DS1(const RegPtrMap*);
    ~LSM9DS1();

    void class_init(uint8_t bus_addr);

    /* Overrides from the BusOpCallback interface */
    int8_t io_op_callahead(BusOp*);
    int8_t io_op_callback(BusOp*);
    int8_t queue_io_job(BusOp*);

    void setSampleRateProfile(uint8_t);
    IMUFault setDesiredState(IMUState);   // Used to set the state the OS wants the IMU class to acheive.
    void     write_test_bytes();
    bool     step_state();      // Used internally to move between states. TODO: Should be private.

    IMUFault init();
    void   reset();           // Reset our state without causing a re-init.

    /* Debug stuff... */
    void dumpDevRegs(StringBuilder*);
    //void dumpDevRegs(StringBuilder*);

    /* Functions called by the IIU */
    int8_t readSensor();      // Call to poll the sensor's registers and take any appropriate action.
    //virtual int8_t enable(bool);             // Pass a boolean to turn the sensor on or off.
    //virtual int8_t set_base_filter_param(uint8_t nu_bw_idx) =0;

    /* State-check functions. Inlined where practical. */
    inline IMUState getState() {            return imu_state;                             }
    inline IMUState desiredState() {        return desired_state;                         }

    inline bool present() {                 return (IMUState::STAGE_0 != getState());        }
    inline bool initPending() {             return ((IMUState::STAGE_1 == getState()) || (IMUState::STAGE_2 == getState()));  }
    inline bool initReadback() {            return (IMUState::STAGE_2 == getState());        }
    inline bool initComplete() {            return (IMUState::STAGE_3 <= getState());        }
    inline bool calibrated() {              return (IMUState::STAGE_4 <= getState());        }
    inline bool idle() {                    return (IMUState::STAGE_4 == getState());        }
    inline bool reading() {                 return (IMUState::STAGE_5 == getState());        }
    inline bool desired_state_attained() {  return (getState() == desiredState());        }

    inline bool profile() {         return _check_flags(IMU_COMMON_FLAG_PROFILING);     };
    inline bool cancel_error() {    return _check_flags(IMU_COMMON_FLAG_CANCEL_ERROR);  };
    inline void profile(bool x) {       _alter_flags(x, IMU_COMMON_FLAG_PROFILING);     };
    inline void cancel_error(bool x) {  _alter_flags(x, IMU_COMMON_FLAG_CANCEL_ERROR);  };

    inline bool autoscale_mag() {   return _check_flags(IMU_COMMON_FLAG_AUTOSCALE_0);   };
    inline void autoscale_mag(bool x) {  _alter_flags(x, IMU_COMMON_FLAG_AUTOSCALE_0);  };
    inline bool autoscale_acc() {   return _check_flags(IMU_COMMON_FLAG_AUTOSCALE_0);   };
    inline void autoscale_acc(bool x) {  _alter_flags(x, IMU_COMMON_FLAG_AUTOSCALE_0);  };
    inline bool autoscale_gyr() {   return _check_flags(IMU_COMMON_FLAG_AUTOSCALE_1);   };
    inline void autoscale_gyr(bool x) {  _alter_flags(x, IMU_COMMON_FLAG_AUTOSCALE_1);  };

    IMUFault request_rescale_mag(uint8_t nu_scale_idx);     // Call to rescale the sensor.
    IMUFault set_sample_rate_mag(uint8_t nu_srate_idx);     // Call to alter sample rate.
    IMUFault set_base_filter_param_mag(uint8_t nu_bw_idx);  // Call to change the bandwidth of the AA filter.

    IMUFault request_rescale_acc(uint8_t nu_scale_idx);     // Call to rescale the sensor.
    IMUFault set_sample_rate_acc(uint8_t nu_srate_idx);     // Call to alter sample rate.
    IMUFault set_base_filter_param_acc(uint8_t nu_bw_idx);  // Call to change the bandwidth of the AA filter.

    IMUFault request_rescale_gyr(uint8_t nu_scale_idx);     // Call to rescale the sensor.
    IMUFault set_sample_rate_gyr(uint8_t nu_srate_idx);     // Call to alter sample rate.
    IMUFault set_base_filter_param_gyr(uint8_t nu_bw_idx);  // Call to change the bandwidth of the AA filter.

    IMUFault irq_drdy(); // When an IRQ signal fires, find the cause and service it.
    IMUFault irq_m();    // When an IRQ signal fires, find the cause and service it.
    IMUFault irq_1();    // When an IRQ signal fires, find the cause and service it.
    IMUFault irq_2();    // When an IRQ signal fires, find the cause and service it.

    IMUFault io_op_callback_mag_read(RegID idx, unsigned int value);
    IMUFault io_op_callback_mag_write(RegID idx, unsigned int value);
    IMUFault io_op_callback_ag_read(RegID idx, unsigned int value);
    IMUFault io_op_callback_ag_write(RegID idx, unsigned int value);


    /* Inlines for the specialized flag duty of get/set class verbosity. */
    inline uint8_t getVerbosity() {
      return (_imu_flags & IMU_COMMON_FLAG_VERBOSITY_MASK);
    };
    inline void  setVerbosity(uint8_t nu) {
      _imu_flags = (nu & IMU_COMMON_FLAG_VERBOSITY_MASK) | (_imu_flags & ~IMU_COMMON_FLAG_VERBOSITY_MASK);
    };

    inline const char* getStateString() {    return getStateString(imu_state);        }
    inline const char* getErrorString() {    return getErrorString(error_condition);  }

    static IMUState getStateByIndex(uint8_t state_idx);
    static const char* getStateString(IMUState);
    static const char* getErrorString(IMUFault);
    static const char* regNameString(RegID);

    static const uint8_t regAddr(RegID);
    static const uint8_t regWidth(RegID);
    static const bool    regWritable(RegID);


    static const GainErrorMap error_map_mag[];
    static const GainErrorMap error_map_acc[];
    static const GainErrorMap error_map_gyr[];

    static const float max_range_vect_mag;
    static const float max_range_vect_acc;
    static const float max_range_vect_gyr;

    static const uint8_t MAXIMUM_GAIN_INDEX_MAG = 4;
    static const uint8_t MAXIMUM_GAIN_INDEX_ACC = 5;
    static const uint8_t MAXIMUM_GAIN_INDEX_GYR = 3;

    static const uint8_t MAXIMUM_RATE_INDEX_MAG = 8;
    static const uint8_t MAXIMUM_RATE_INDEX_AG  = 7;

    static const UpdateRate2Hertz rate_settings_mag[MAXIMUM_RATE_INDEX_MAG];
    static const UpdateRate2Hertz rate_settings_acc[MAXIMUM_RATE_INDEX_AG];
    static const UpdateRate2Hertz rate_settings_gyr[MAXIMUM_RATE_INDEX_AG];


  private:
    const RegPtrMap* _ptr_map;
    uint8_t BUS_ADDR;  // What is our address on the bus? TODO: const

    // TODO: r1 simplified things a great deal. All these members can probably DIAF.
    RegID IDX_T0; // TODO: Ought to be const if here at all.
    RegID IDX_T1; // TODO: Ought to be const if here at all.
    RegID IDX_ID; // TODO: Ought to be const if here at all.

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
    IMUFault  error_condition  = IMUFault::NO_ERROR;

    IMUState  imu_state        = IMUState::STAGE_0;
    IMUState  desired_state    = IMUState::STAGE_0;

    StringBuilder local_log;


    uint8_t scale_mag            = 0;     // What scale is the sensor operating at? This is an index.
    uint8_t scale_acc            = 0;     // What scale is the sensor operating at? This is an index.
    uint8_t scale_gyr            = 0;     // What scale is the sensor operating at? This is an index.

    uint8_t update_rate_mag      = 0;     // Index to the update-rate array.
    uint8_t update_rate_acc      = 0;     // Index to the update-rate array.
    uint8_t update_rate_gyr      = 0;     // Index to the update-rate array.

    uint16_t discards_remain_mag = 0;     // If we know we need to discard samples...
    uint16_t discards_remain_acc = 0;     // If we know we need to discard samples...
    uint16_t discards_remain_gyr = 0;     // If we know we need to discard samples...
    uint32_t discards_total_mag  = 0;     // Track how many discards we've ASKED for.
    uint32_t discards_total_acc  = 0;     // Track how many discards we've ASKED for.
    uint32_t discards_total_gyr  = 0;     // Track how many discards we've ASKED for.

    Vector3<float> last_val_mag;
    Vector3<float> last_val_acc;
    Vector3<float> last_val_gyr;

    Vector3<int16_t> sample_backlog_mag[32];
    Vector3<int16_t> sample_backlog_acc[32];
    Vector3<int16_t> sample_backlog_gyr[32];

    Vector3<int16_t> noise_floor_mag;
    Vector3<int16_t> noise_floor_acc;
    Vector3<int16_t> noise_floor_gyr;

    /* These are higher-level fxns that are used as "macros" for specific patterns of */
    /*   register access. Common large-scale operations should go here.               */
    IMUFault writeRegister(RegID idx, unsigned int nu_val);

    unsigned int regValue(RegID);
    /* This is the end of the low-level functions.                                    */

    IMUFault identity_check();
    bool integrity_check();

    bool is_setup_completed();

    /**
    * Sets the current IMU state without blowing away the high bits in the state member.
    */
    inline void set_state(IMUState nu) {     imu_state = nu;   }

    /* Inlines for altering and reading the flags. */
    inline void _alter_flags(bool en, uint16_t mask) {
      _imu_flags = (en) ? (_imu_flags | mask) : (_imu_flags & ~mask);
    };
    inline bool _check_flags(uint16_t mask) {
      return (mask == (_imu_flags & mask));
    };

    inline bool power_to_mag() {   return _check_flags(IMU_COMMON_FLAG_MAG_POWERED);   };
    inline void power_to_mag(bool x) {  _alter_flags(x, IMU_COMMON_FLAG_MAG_POWERED);  };
    inline bool power_to_acc() {   return _check_flags(IMU_COMMON_FLAG_ACC_POWERED);   };
    inline void power_to_acc(bool x) {  _alter_flags(x, IMU_COMMON_FLAG_ACC_POWERED);  };
    inline bool power_to_gyr() {   return _check_flags(IMU_COMMON_FLAG_GYR_POWERED);   };
    inline void power_to_gyr(bool x) {  _alter_flags(x, IMU_COMMON_FLAG_GYR_POWERED);  };

    int8_t calibrate_from_data_mag();
    int8_t calibrate_from_data_ag();

    int8_t collect_reading_mag();
    int8_t collect_reading_acc();
    int8_t collect_reading_gyr();
    int8_t collect_reading_temperature();
};

#endif // __LSM9DS1_MERGED_H__
