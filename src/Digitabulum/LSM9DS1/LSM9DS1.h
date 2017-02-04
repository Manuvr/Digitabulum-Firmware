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

#include "../ManuLegend/SensorFrame.h"


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
typedef struct {
  const float hertz;      // Frequency
  const float ts_delta;   // Period (in seconds)
} UpdateRate2Hertz;


/* We use the struct to map between scales and error-rates. */
typedef struct {
  const uint16_t scale;     // This is the maximum magnatude of the sensor reading at the given gain.
  const float    per_lsb;   // Each LSB counts for this many of whatever unit.
  const float    error;     // Given in the sensor's native unit. Each reading at this scale has this error.
} GainErrorMap;


/*
* Internally-used register IDs. We need this abstraction because the addresses
*   within a given package are not disjoint sets.
* The order corrosponds to the order of the named register's occurance in the device.
*
* NOTE: These values are NOT register addresses, they are indicies. The given
*   order, and the fact that the first valid register starts at 0 is an
*   assumption made throughout this class, as well as RegPtrMap.
*   Magnetometer registers occur first because the CPLD is organized that way.
* NOTE: In order to get away with using the restricted list, the sensors MUST be
*   configured to allow multiple sequential register access. The means for doing
*   this vary between mag/ag.
*/
enum class RegID {
  M_OFFSET_X = 0x00,  // 16-bit offset registers
  M_OFFSET_Y,         // 16-bit offset registers  TODO: Condense into X?
  M_OFFSET_Z,         // 16-bit offset registers  TODO: Condense into X?
  M_WHO_AM_I,
  M_CTRL_REG1,
  M_CTRL_REG2,
  M_CTRL_REG3,
  M_CTRL_REG4,
  M_CTRL_REG5,
  M_STATUS_REG,
  M_DATA_X,           // 16-bit data registers
  M_DATA_Y,           // 16-bit data registers  TODO: Condense into X?
  M_DATA_Z,           // 16-bit data registers  TODO: Condense into X?
  M_INT_CFG,
  M_INT_SRC,
  M_INT_TSH,          // 16-bit threshold register
  AG_ACT_THS,
  AG_ACT_DUR,
  A_INT_GEN_CFG,
  A_INT_GEN_THS_X,    // 8-bit threshold registers
  A_INT_GEN_THS_Y,    // 8-bit threshold registers  TODO: Condense into X?
  A_INT_GEN_THS_Z,    // 8-bit threshold registers  TODO: Condense into X?
  A_INT_GEN_DURATION,
  G_REFERENCE,
  AG_INT1_CTRL,
  AG_INT2_CTRL,
  AG_WHO_AM_I,        // TODO: Condense into M_WHO_AM_I?
  G_CTRL_REG1,
  G_CTRL_REG2,
  G_CTRL_REG3,
  G_ORIENT_CFG,
  G_INT_GEN_SRC,
  AG_DATA_TEMP,       // 16-bit temperature register (11-bit)
  AG_STATUS_REG,
  G_DATA_X,           // 16-bit gyro data registers
  G_DATA_Y,           // 16-bit gyro data registers  TODO: Condense into X?
  G_DATA_Z,           // 16-bit gyro data registers  TODO: Condense into X?
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
  A_DATA_Y,           // 16-bit accelerometer data registers  TODO: Condense into X?
  A_DATA_Z,           // 16-bit accelerometer data registers  TODO: Condense into X?
  AG_FIFO_CTRL,
  AG_FIFO_SRC,
  G_INT_GEN_CFG,
  G_INT_GEN_THS_X,    // 16-bit threshold registers
  G_INT_GEN_THS_Y,    // 16-bit threshold registers  TODO: Condense into X?
  G_INT_GEN_THS_Z,    // 16-bit threshold registers  TODO: Condense into X?
  G_INT_GEN_DURATION
};



/*******************************************************************************
* This is the const object that is passed into the LSM9DS1 constructor. Its
*   values are pointers to the memory that represents the LSM9DS1 registers.
*******************************************************************************/
class RegPtrMap {
  // NOTE: This class should lose ptr members over time as limits are tested.
  public:
    RegPtrMap(
      const uint8_t   idx,
      const uint8_t*  ag_activity,
      const uint8_t*  ag_ctrl1_3,
      const uint8_t*  ag_ctrl6_7,
      const uint8_t*  ag_status,
      const uint8_t*  fifo_src,
      const int16_t*  g_thresholds,
      const uint8_t*  g_irq_dur,
      const int16_t*  m_offsets,
      const uint8_t*  m_ctrl2,
      const uint8_t*  m_status,
      const uint8_t*  m_irq_src,
      const uint16_t* m_thresholds
    ) :
      AG_ACT((idx * 2) + ag_activity),
      AG_CTRL1_3((idx * 3) + ag_ctrl1_3),
      AG_CTRL6_7((idx * 2) + ag_ctrl6_7),
      AG_STATUS(idx + ag_status),
      FIFO_LVLS(idx + fifo_src),
      G_INT_THS((idx * 6) + g_thresholds),  // 16-bit: Horrid.
      G_INT_DUR(idx + g_irq_dur),
      M_OFFSETS((idx * 6) + m_offsets),     // 16-bit: Horrid.
      M_CTRL(idx + m_ctrl2),
      M_STATUS(idx + m_status),
      M_INT_SRC(idx + m_irq_src),
      M_INT_TSH((idx * 2) + m_thresholds)   // 16-bit: Horrid.
    {};

    const uint8_t* regPtr(RegID) const;

    static RegID regIdFromAddr(uint8_t, uint8_t);
    static const uint8_t regAddr(RegID);
    static const uint8_t regWidth(RegID);
    static const bool    regWritable(RegID);
    static const char*   regNameString(RegID);


  private:
    // Some registers are not included in this list if their function can be
    //   handled entirely within ManuManager.
    const uint8_t* AG_ACT;      // AG_ACT_THS, AG_ACT_DUR
    const uint8_t* AG_BLOCK_0;  // A_INT_GEN_CFG, A_INT_GEN_THS_X, A_INT_GEN_THS_Y, A_INT_GEN_THS_Z, A_INT_GEN_DURATION, G_REFERENCE,
    const uint8_t* AG_CTRL1_3;  // G_CTRL_REG1, G_CTRL_REG2, G_CTRL_REG3
    const uint8_t* AG_CTRL6_7;  // A_CTRL_REG6, A_CTRL_REG7
    const uint8_t* AG_STATUS;   // AG_STATUS_REG
    const uint8_t* FIFO_LVLS;   // AG_FIFO_SRC

    const int16_t* G_INT_THS;   // 16-bit: G_INT_GEN_THS_X, G_INT_GEN_THS_Y, G_INT_GEN_THS_Z
    const uint8_t* G_INT_DUR;   // G_INT_GEN_DURATION

    const int16_t* M_OFFSETS;   // 16-bit: M_OFFSET_X, M_OFFSET_Y, M_OFFSET_Z
    const uint8_t* M_CTRL;      // M_CTRL_REG2
    const uint8_t* M_STATUS;    // M_STATUS_REG
    const uint8_t* M_INT_SRC;   // M_INT_SRC
    const uint16_t* M_INT_TSH;  // 16-bit: M_INT_TSH
};


/*******************************************************************************
* The LSM9DS1 class. Acc, Gyr, and Mag are all handled here, and the work of
*   distinguishing between them falls on the CPLD hardware and ManuManager.
*******************************************************************************/
class LSM9DS1 {
  public:
    LSM9DS1(const RegPtrMap*);
    ~LSM9DS1();


    void setSampleRateProfile(uint8_t);
    IMUFault setDesiredState(IMUState);   // Used to set the state the OS wants the IMU class to acheive.
    void     write_test_bytes();
    bool     step_state();      // Used internally to move between states. TODO: Should be private.

    IMUFault init();
    void     reset();           // Reset our state without causing a re-init.

    /* Debug stuff... */
    void dumpDevRegs(StringBuilder*);
    //void dumpDevRegs(StringBuilder*);

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

    inline float scaleA() {  return error_map_acc[scale_acc].per_lsb;  };
    inline float scaleG() {  return error_map_gyr[scale_gyr].per_lsb;  };
    inline float scaleM() {  return error_map_mag[scale_mag].per_lsb;  };

    inline float deltaT_I() {  return rate_settings_i[update_rate_i].ts_delta;  };
    inline float deltaT_M() {  return rate_settings_m[update_rate_m].ts_delta;  };


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

    IMUFault proc_register_read(RegID idx);
    IMUFault proc_register_write(RegID idx);


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

    static const UpdateRate2Hertz rate_settings_m[MAXIMUM_RATE_INDEX_MAG];
    static const UpdateRate2Hertz rate_settings_i[MAXIMUM_RATE_INDEX_AG];


  private:
    const RegPtrMap* _ptr_map;

    uint32_t  discards_total_i    = 0;     // Track how many discards we've ASKED for.
    uint32_t  discards_total_m    = 0;     // Track how many discards we've ASKED for.
    uint16_t  discards_remain_i   = 0;     // If we know we need to discard samples...
    uint16_t  discards_remain_m   = 0;     // If we know we need to discard samples...

    uint16_t  _imu_flags          = 1;     // Default verbosity of 1.

    uint8_t   io_test_val_0       = 0;     //
    uint8_t   io_test_val_1       = 0;     //

    uint8_t   sb_next_read        = 0;
    uint8_t   sb_next_write       = 0;

    int8_t    base_filter_param   = 0;

    IMUFault  error_condition     = IMUFault::NO_ERROR;

    IMUState  imu_state           = IMUState::STAGE_0;
    IMUState  desired_state       = IMUState::STAGE_0;

    uint8_t   scale_mag           = 0;     // Index to the scale array.
    uint8_t   scale_acc           = 0;     // Index to the scale array.
    uint8_t   scale_gyr           = 0;     // Index to the scale array.

    uint8_t   update_rate_i       = 0;     // Index to the update-rate array.
    uint8_t   update_rate_m       = 0;     // Index to the update-rate array.

    IMUFault writeRegister(RegID idx, unsigned int nu_val);
    unsigned int regValue(RegID);


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
};

#endif // __LSM9DS1_MERGED_H__
