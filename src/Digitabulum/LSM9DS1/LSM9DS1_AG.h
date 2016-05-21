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

#ifndef __LSM9DS1_AG_H
#define __LSM9DS1_AG_H

#include "LSM9DSx.h"

// BIG FAT WARNING: These values are NOT register addresses, they are
//   indicies. This is the order of the register's occurance in the device.
#define LSM9DS1_AG_ACT_THS            0x00
#define LSM9DS1_AG_ACT_DUR            0x01
#define LSM9DS1_A_INT_GEN_CFG         0x02
#define LSM9DS1_A_INT_GEN_THS_X       0x03  // 8-bit threshold registers
#define LSM9DS1_A_INT_GEN_THS_Y       0x04  // 8-bit threshold registers
#define LSM9DS1_A_INT_GEN_THS_Z       0x05  // 8-bit threshold registers
#define LSM9DS1_A_INT_GEN_DURATION    0x06
#define LSM9DS1_G_REFERENCE           0x07
#define LSM9DS1_AG_INT1_CTRL          0x08
#define LSM9DS1_AG_INT2_CTRL          0x09
#define LSM9DS1_AG_WHO_AM_I           0x0A
#define LSM9DS1_G_CTRL_REG1           0x10
#define LSM9DS1_G_CTRL_REG2           0x11
#define LSM9DS1_G_CTRL_REG3           0x12
#define LSM9DS1_G_ORIENT_CFG          0x13
#define LSM9DS1_G_INT_GEN_SRC         0x14
#define LSM9DS1_AG_DATA_TEMP          0x15  // 16-bit temperature register (11-bit)
#define LSM9DS1_AG_STATUS_REG         0x16
#define LSM9DS1_G_DATA_X              0x17  // 16-bit gyro data registers
#define LSM9DS1_G_DATA_Y              0x18  // 16-bit gyro data registers
#define LSM9DS1_G_DATA_Z              0x19  // 16-bit gyro data registers
#define LSM9DS1_AG_CTRL_REG4          0x1A
#define LSM9DS1_A_CTRL_REG5           0x1B
#define LSM9DS1_A_CTRL_REG6           0x1C
#define LSM9DS1_A_CTRL_REG7           0x1D
#define LSM9DS1_AG_CTRL_REG8          0x1E
#define LSM9DS1_AG_CTRL_REG9          0x1F
#define LSM9DS1_AG_CTRL_REG10         0x20
#define LSM9DS1_A_INT_GEN_SRC         0x21
#define LSM9DS1_AG_STATUS_REG_ALT     0x22
#define LSM9DS1_A_DATA_X              0x23  // 16-bit accelerometer data registers
#define LSM9DS1_A_DATA_Y              0x24  // 16-bit accelerometer data registers
#define LSM9DS1_A_DATA_Z              0x25  // 16-bit accelerometer data registers
#define LSM9DS1_AG_FIFO_CTRL          0x26
#define LSM9DS1_AG_FIFO_SRC           0x27
#define LSM9DS1_G_INT_GEN_CFG         0x28
#define LSM9DS1_G_INT_GEN_THS_X       0x29  // 16-bit threshold registers
#define LSM9DS1_G_INT_GEN_THS_Y       0x2A  // 16-bit threshold registers
#define LSM9DS1_G_INT_GEN_THS_Z       0x2B  // 16-bit threshold registers
#define LSM9DS1_G_INT_GEN_DURATION    0x2C



/****************************************************************************************************
* Inertial aspect                                                                                   *
****************************************************************************************************/

/**
* The hardware driver for the accelerometer and the gyroscope in the LSM9DS1.
*
* Because this is a three-part sensor, it's going to get a bit messy in here...
*
* TODO: The temperature sensor directly feeds the error-correction mechanism. The data is
*   also fed downstream to the IIU class for representation, but the error reports
*   for InertialMeasurement ought to be done within this class.
*/
class LSM9DS1_AG : public LSM9DSx_Common {
  public:
    bool    autoscale_acc    = false;  // Should the class autoscale?
    bool    autoscale_gyr    = false;  // Should the class autoscale?

    LSM9DS1_AG(uint8_t bus_addr, IIU* _integrator);
    ~LSM9DS1_AG(void);


    /* Specific to this class */
    int8_t irq_0();    // When an IRQ signal fires, find the cause and service it.
    int8_t irq_1();    // When an IRQ signal fires, find the cause and service it.

    int8_t request_rescale_acc(uint8_t nu_scale_idx);     // Call to rescale the sensor.
    int8_t set_sample_rate_acc(uint8_t nu_srate_idx);     // Call to alter sample rate.
    int8_t set_base_filter_param_acc(uint8_t nu_bw_idx);  // Call to change the bandwidth of the AA filter.

    int8_t request_rescale_gyr(uint8_t nu_scale_idx);     // Call to rescale the sensor.
    int8_t set_sample_rate_gyr(uint8_t nu_srate_idx);     // Call to alter sample rate.
    int8_t set_base_filter_param_gyr(uint8_t nu_bw_idx);  // Call to change the bandwidth of the AA filter.

    /* Overrides from LSM9DSx class */
    int8_t bulk_refresh();
    void   reset();
    int8_t readSensor();

    /* Overrides from the SPICallback interface */
    virtual int8_t spi_op_callback(SPIBusOp*);

    /* Debug stuff... */
    virtual void dumpDevRegs(StringBuilder*);
    virtual void dumpPreformedElements(StringBuilder*);


    static const float max_range_vect_acc;
    static const float max_range_vect_gyr;

    static const GainErrorMap error_map_acc[];
    static const GainErrorMap error_map_gyr[];



  protected:
    bool   is_setup_completed();
    int8_t configure_sensor();


  private:
    Vector3<int16_t> sample_backlog_acc[32];
    Vector3<int16_t> sample_backlog_gyr[32];

    bool    power_to_acc         = false;  // Sensor powered on?
    bool    power_to_gyr         = false;  // Sensor powered on?

    uint8_t scale_acc            = 0;      // What scale is the sensor operating at? This is an index.
    uint8_t scale_gyr            = 0;      // What scale is the sensor operating at? This is an index.

    uint8_t update_rate_acc      = 0;      // Index to the update-rate array.
    uint8_t update_rate_gyr      = 0;      // Index to the update-rate array.

    uint16_t discards_remain_acc = 0;      // If we know we need to discard samples...
    uint16_t discards_remain_gyr = 0;      // If we know we need to discard samples...

    uint32_t discards_total_acc  = 0;      // Track how many discards we've ASKED for.
    uint32_t discards_total_gyr  = 0;      // Track how many discards we've ASKED for.

    SPIBusOp preformed_busop_irq_0;
    SPIBusOp preformed_busop_irq_1;

    SPIBusOp preformed_busop_read_acc;
    SPIBusOp preformed_busop_read_gyr;


    Vector3<float> last_val_acc;
    Vector3<float> last_val_gyr;

    Vector3<int16_t> noise_floor_acc;
    Vector3<int16_t> noise_floor_gyr;


    int8_t collect_reading_acc();
    int8_t collect_reading_gyr();
    int8_t collect_reading_temperature();

    int8_t calibrate_from_data();


    static const uint8_t MAXIMUM_GAIN_INDEX_ACC = 5;
    static const uint8_t MAXIMUM_GAIN_INDEX_GYR = 3;

    static const uint8_t MAXIMUM_RATE_INDEX_AG  = 7;
    static const UpdateRate2Hertz rate_settings_acc[MAXIMUM_RATE_INDEX_AG];
    static const UpdateRate2Hertz rate_settings_gyr[MAXIMUM_RATE_INDEX_AG];
};

#endif  // __LSM9DS1_AG_H
