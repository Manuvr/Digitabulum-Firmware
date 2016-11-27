/*
File:   LSM9DS1_M.h
Author: J. Ian Lindsay
Date:   2015.05.18

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

#ifndef __LSM9DS1_MAG_H
#define __LSM9DS1_MAG_H

#include "LSM9DSx.h"

// BIG FAT WARNING: These values are NOT register addresses, they are
//   indicies. This is the order of the register's occurance in the device.
#define LSM9DS1_M_OFFSET_X         0x00  // 16-bit offset registers
#define LSM9DS1_M_OFFSET_Y         0x01  // 16-bit offset registers
#define LSM9DS1_M_OFFSET_Z         0x02  // 16-bit offset registers
#define LSM9DS1_M_WHO_AM_I         0x03
#define LSM9DS1_M_CTRL_REG1        0x04
#define LSM9DS1_M_CTRL_REG2        0x05
#define LSM9DS1_M_CTRL_REG3        0x06
#define LSM9DS1_M_CTRL_REG4        0x07
#define LSM9DS1_M_CTRL_REG5        0x08
#define LSM9DS1_M_STATUS_REG       0x09
#define LSM9DS1_M_DATA_X           0x0A  // 16-bit data registers
#define LSM9DS1_M_DATA_Y           0x0B  // 16-bit data registers
#define LSM9DS1_M_DATA_Z           0x0C  // 16-bit data registers
#define LSM9DS1_M_INT_CFG          0x0D
#define LSM9DS1_M_INT_SRC          0x0E
#define LSM9DS1_M_INT_TSH          0x0F  // 16-bit threshold register



/****************************************************************************************************
* Magnetic aspect                                                                                   *
****************************************************************************************************/

/**
* The hardware driver for the magnetometer in the LSM9DS1.
*/
class LSM9DS1_M : public LSM9DSx_Common {
  public:
    LSM9DS1_M(uint8_t bus_addr, IIU* _integrator);
    ~LSM9DS1_M(void);

    /* Specific to this class */
    inline bool autoscale_mag() {   return _check_flags(IMU_COMMON_FLAG_AUTOSCALE_0);   };
    inline void autoscale_mag(bool x) {  _alter_flags(x, IMU_COMMON_FLAG_AUTOSCALE_0);  };

    int8_t irq();    // When an IRQ signal fires, find the cause and service it.

    int8_t request_rescale_mag(uint8_t nu_scale_idx);     // Call to rescale the sensor.
    int8_t set_sample_rate_mag(uint8_t nu_srate_idx);     // Call to alter sample rate.
    int8_t set_base_filter_param_mag(uint8_t nu_bw_idx);  // Call to change the bandwidth of the AA filter.

    /* Overrides from LSM9DSx class */
    int8_t bulk_refresh();
    void   reset();
    int8_t readSensor(void);

    /* Overrides from the SPICallback interface */
    int8_t io_op_callback(BusOp*);

    /* Debug stuff... */
    virtual void dumpDevRegs(StringBuilder*);
    virtual void dumpPreformedElements(StringBuilder*);


    static const GainErrorMap error_map_mag[];
    static const float max_range_vect_mag;



  protected:
    bool   is_setup_completed();
    int8_t configure_sensor();
    const char* imu_type() {   return "MAG"; };


  private:
    Vector3<int16_t> sample_backlog_mag[32];

    uint8_t scale_mag            = 0;     // What scale is the sensor operating at? This is an index.
    uint8_t update_rate_mag      = 0;     // Index to the update-rate array.

    uint16_t discards_remain_mag = 0;     // If we know we need to discard samples...
    uint32_t discards_total_mag  = 0;     // Track how many discards we've ASKED for.

    SPIBusOp preformed_busop_irq_mag;
    SPIBusOp preformed_busop_read_mag;

    Vector3<float> last_val_mag;

    Vector3<int16_t> noise_floor_mag;

    //uint8_t cutoff_idx;                   // The sensor's onboard filter cutoff freq.
    //float cutoff;                         // The sensor's onboard filter cutoff freq.
    inline bool power_to_mag() {   return _check_flags(IMU_COMMON_FLAG_MAG_POWERED);   };
    inline void power_to_mag(bool x) {  _alter_flags(x, IMU_COMMON_FLAG_MAG_POWERED);  };

    int8_t collect_reading_mag();

    int8_t calibrate_from_data();

    static const uint8_t MAXIMUM_RATE_INDEX_MAG = 8;
    static const uint8_t MAXIMUM_GAIN_INDEX_MAG = 4;
    static const UpdateRate2Hertz rate_settings_mag[MAXIMUM_RATE_INDEX_MAG];
};

#endif  // __LSM9DS1_MAG_H
