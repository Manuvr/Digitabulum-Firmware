/*
File:   IIU.h
Author: J. Ian Lindsay
Date:   2014.03.31

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


An IIU is a non-abstract class that tracks measurements from an IMU and
  maintains positional data (by integrating the readings from the IMU).

This class will work with what data it is given. But in the most-leveraged case, this
  class would be fed one of each sensor type supported. The GPS would be used to correct
  gross drift over time, and the dead-reckoning sensors would be used to ignore GPS
  jitter.
In the case where only one sensor is fed to this class (suppose it's a gyro), only the
  orientation would be integratable, so that is all that would be reported in such a case.


An IIU can be fed any number of these data:
  1) Rotation (via a gyro)
  2) Acceleration (via an accelerometer)
  3) Magnetic field vector
  4) GPS data

Data from magnetometers and GPS devices is used to establish error-rates
  and divergence data (and optionally correction).


*/


#ifndef __IIU_CLASS_H__
#define __IIU_CLASS_H__

#include "LSM9DS1_M.h"
#include "LSM9DS1_AG.h"
#include <stdarg.h>
#include <Kernel.h>
#include <DataStructures/InertialMeasurement.h>
#include <DataStructures/Quaternion.h>

class LSM9DSx_Common;   // Forward declaration of the LSM9DSx_Common class.


/*
* push_measurement() takes a flag that is used to store important qualifications about the sample.
* These flags are stored as bitwise flags so we can note many things at once.
*/
#define IMU_FLAG_UNSPECIFIED_DATA  0b11111111  // Matches capabilities flag for magnetometer data.
#define IMU_FLAG_ACCEL_DATA        0b00000000  // Matches capabilities flag for accelerometer data.
#define IMU_FLAG_GYRO_DATA         0b00000001  // Matches capabilities flag for gyro data.
#define IMU_FLAG_MAG_DATA          0b00000010  // Matches capabilities flag for magnetometer data.
#define IMU_FLAG_GRAVITY_DATA      0b00000100  // Matches capabilities flag for gravity data.
#define IMU_FLAG_BEARING_DATA      0b00001000  // Matches capabilities flag for bearing at calibration.




/*
* Sometimes we need to flag a sample to indicate that some important event happened.
* We do this with bitwise flags so that we can use the same field to store many events.
*/
#define IIU_NO_EVENT           0b0000000000000000            // Nothing special about this sample.
#define IIU_INFLECTION_POINT   0b0000000000000001            // When the sensor reading indicates a direction change.
#define IIU_GAIN_SHIFT         0b0000000000001000            // When this sample was taken with a gain that differed from the last.

#define IIU_CO_IIU_CORRECTION  0b0010000000000000            // When this sample was corrected from an consensus of redundant IMU/IIUs.
#define IIU_MAG_CORRECTION     0b0100000000000000            // When this sample was corrected from magneteometer data.
#define IIU_GPS_CORRECTION     0b1000000000000000            // When this sample was corrected by a GPS fix.


/*
* List of parameters (software defined registers) that this sensor can deal with.
*/
#define IIU_PARAM_EC_DEV_GPS           0x1000   // This class can take a GPS device to correct for error.
#define IIU_PARAM_TEMPERATURE          0x1001   // This class can use temperature data to predict its error.
#define IIU_PARAM_EC_DEV_MAGNETOMETER  0x1002   // This class can take a magnetometer to correct for error.
#define IIU_PARAM_FRAME_QUEUE_LENGTH   0x1010   // How many frames should we delay for the sake of DrifCor, or feeding to a host-bound buffer?
#define IIU_PARAM_3SPACE_POSITION      0x1011   // This register sets the IIU's position in 3-space.
#define IIU_PARAM_REPORTING_MODE       0x1012   // The IIU can report in absolute terms, or in relative terms.



#define IIU_DATA_HANDLING_UNITS_METRIC     0x00010000

#define IIU_DATA_HANDLING_FIND_POS         0x00100000
#define IIU_DATA_HANDLING_FIND_VEL         0x00200000

#define IIU_DATA_HANDLING_MAG_NULL_BEARING 0x00400000  //
#define IIU_DATA_HANDLING_MAG_CORRECT_SPH  0x00800000  //

#define IIU_DATA_HANDLING_PROFILING        0x01000000
#define IIU_DATA_HANDLING_NULL_GYRO_ERROR  0x02000000
#define IIU_DATA_HANDLING_SMART_MAG_DROP   0x04000000  // If enabled, causes a large magnetometer reading to be DQ'd from AHRS.
#define IIU_DATA_HANDLING_CLEAN_MAG_ZERO   0x08000000  //

#define IIU_DATA_HANDLING_LEGEND_WRITABLE  0x10000000
#define IIU_DATA_HANDLING_PROC_QUAT        0x20000000
#define IIU_DATA_HANDLING_RANGE_BIND       0x40000000
#define IIU_DATA_HANDLING_NULLIFY_GRAVITY  0x80000000


#define IIU_STANDARD_GRAVITY           9.80665f // This is Earth's gravity at sea-level, in m/s^2

/**
* This is the Inertial Integration Unit class. It has the following responsibilities:
*   1) Bind together several discrete logical IMU sensors (gyro / accelerometer / mag) into a
*        unified representation.
*   2) Manage the low-level activities of those sensors according to legend specification.
*   3) Accumulate and track error rates to inform the drift corrector.
*   4) Integrate inertial changes into an idea of position and orientation.
*/
class IIU {
  public:
    float beta;
    float grav_scalar = 0.0f;


    IIU();
    ~IIU(void);


    int8_t init(void);
    int8_t readSensor(void);

    int8_t setParameter(uint16_t reg, int len, uint8_t*);  // Used to set operational parameters for the sensor.
    int8_t getParameter(uint16_t reg, int len, uint8_t*);  // Used to read operational parameters from the sensor.

    /* Specific to this class */
    inline float getTemperature() {      return *(_ptr_temperature);  }
    void setTemperature(float);
    uint32_t totalSamples();

    inline int8_t position() {  return pos_id;   }

    void reset();
    void sync();

    int8_t irq_ag0();
    int8_t irq_ag1();
    int8_t irq_m();

    void setSampleRate(uint8_t idx);

    bool state_pass_through(uint8_t);

    void setVerbosity(int8_t);
    inline int8_t getVerbosity() { return verbosity; };
    void printDebug(StringBuilder*);
    void printBrief(StringBuilder*);


    /* These are meant to be called from the IMUs. */
    int8_t pushMeasurement(uint8_t, float x, float y, float z, float delta_t);
    int8_t state_change_notice(LSM9DSx_Common* imu_ptr, uint8_t p_state, uint8_t c_state);

    /* These are meant to be called from a Legend. */
    void setPositionAndAddress(uint8_t nu_pos, uint8_t imu_addr, uint8_t mag_addr);
    void setOperatingState(uint8_t);
    void printLastFrame(StringBuilder *output);
    void dumpPreformedElements(StringBuilder*);

    void setSampleRateProfile(uint8_t);

    void assign_legend_pointers(
      void* acc,
      void* gyr,
      void* mag,
      void* vel,
      void* null_g,
      void* pos,
      void* quat,
      void* temperature,
      void* sample_count_acc,
      void* sample_count_gyr,
      void* sample_count_mag,
      void* sample_count_temp
    );

    uint8_t MadgwickQuaternionUpdate();

    void dumpPointers(StringBuilder*);

    inline bool isDirty() {   return (dirty_acc||dirty_gyr||dirty_mag);    }   // Has the sensor been updated?
    inline bool isQuatDirty() {   return (dirty_acc & dirty_gyr);          }   // Has the sensor been updated?
    inline bool has_quats_left() {       return (quat_queue.size() > 0);   }


    /*
    * Accessors for autoscaling.
    */
    void enableAutoscale(uint8_t s_type, bool enabled);
    inline void enableAutoscale(bool enabled) {
      enableAutoscale((IMU_FLAG_MAG_DATA | IMU_FLAG_ACCEL_DATA | IMU_FLAG_GYRO_DATA), enabled);
    };

    /*
    * Accessors for profiling.
    * TODO: Inline these
    */
    inline bool enableProfiling() {         return (data_handling_flags & IIU_DATA_HANDLING_PROFILING);  }
    bool enableProfiling(bool en);

    /*
    * Accessors for gravity cancelation.
    */
    inline bool nullifyGravity() {         return ((_ptr_null_grav) && (data_handling_flags & IIU_DATA_HANDLING_NULLIFY_GRAVITY));  }
    bool nullifyGravity(bool en);

    /*
    * Accessors for quaternion processing.
    */
    inline bool processQuats() {         return (data_handling_flags & IIU_DATA_HANDLING_PROC_QUAT);  }
    inline void processQuats(bool en) {
      data_handling_flags = (en) ? (data_handling_flags | IIU_DATA_HANDLING_PROC_QUAT) : (data_handling_flags & ~(IIU_DATA_HANDLING_PROC_QUAT));
    }

    /*
    * Accessors for velocity processing.
    */
    inline bool findVelocity() {         return (data_handling_flags & IIU_DATA_HANDLING_FIND_VEL);  }
    inline void findVelocity(bool en) {
      data_handling_flags = (en) ? (data_handling_flags | IIU_DATA_HANDLING_FIND_VEL) : (data_handling_flags & ~(IIU_DATA_HANDLING_FIND_VEL));
    }

    /*
    * Accessors for position processing.
    */
    inline bool trackPosition() {         return (data_handling_flags & IIU_DATA_HANDLING_FIND_POS);  }
    inline void trackPosition(bool en) {
      data_handling_flags = (en) ? (data_handling_flags | IIU_DATA_HANDLING_FIND_POS) : (data_handling_flags & ~(IIU_DATA_HANDLING_FIND_POS));
    }

    /*
    * Accessors for quaternion processing.
    */
    inline bool nullGyroError() {         return (data_handling_flags & IIU_DATA_HANDLING_NULL_GYRO_ERROR);  }
    bool nullGyroError(bool en);

    /*
    * Accessors for range-binding output.
    */
    inline bool rangeBind() {         return (data_handling_flags & IIU_DATA_HANDLING_RANGE_BIND);  }
    inline void rangeBind(bool en) {
      data_handling_flags = (en) ? (data_handling_flags | IIU_DATA_HANDLING_RANGE_BIND) : (data_handling_flags & ~(IIU_DATA_HANDLING_RANGE_BIND));
    }

    /*
    * Accessors for magnetometer drop.
    */
    inline bool dropObviousBadMag() {         return (data_handling_flags & IIU_DATA_HANDLING_SMART_MAG_DROP);  }
    inline void dropObviousBadMag(bool en) {
      data_handling_flags = (en) ? (data_handling_flags | IIU_DATA_HANDLING_SMART_MAG_DROP) : (data_handling_flags & ~(IIU_DATA_HANDLING_SMART_MAG_DROP));
    }

    /*
    * Accessors for magnetometer bearing nullification.
    */
    inline bool nullifyBearing() {         return (data_handling_flags & IIU_DATA_HANDLING_MAG_NULL_BEARING);  }
    inline void nullifyBearing(bool en) {
      data_handling_flags = (en) ? (data_handling_flags | IIU_DATA_HANDLING_MAG_NULL_BEARING) : (data_handling_flags & ~(IIU_DATA_HANDLING_MAG_NULL_BEARING));
    }

    /*
    * Accessors for magnetometer spherical abberation correction.
    */
    inline bool correctSphericalAbberation() {         return (data_handling_flags & IIU_DATA_HANDLING_MAG_CORRECT_SPH);  }
    inline void correctSphericalAbberation(bool en) {
      data_handling_flags = (en) ? (data_handling_flags | IIU_DATA_HANDLING_MAG_CORRECT_SPH) : (data_handling_flags & ~(IIU_DATA_HANDLING_MAG_CORRECT_SPH));
    }

    /*
    * Accessors for magnetometer drop.
    */
    inline bool cleanMagZero() {         return (data_handling_flags & IIU_DATA_HANDLING_CLEAN_MAG_ZERO);  }
    inline void cleanMagZero(bool en) {
      data_handling_flags = (en) ? (data_handling_flags | IIU_DATA_HANDLING_CLEAN_MAG_ZERO) : (data_handling_flags & ~(IIU_DATA_HANDLING_CLEAN_MAG_ZERO));
    }


    inline void setAccelBaseFiler(uint8_t nu) {  if (imu_ag) imu_ag->set_base_filter_param(nu);  };
    inline void setGyroBaseFiler(uint8_t nu) {   if (imu_m) imu_m->set_base_filter_param(nu);    };


    /*
    * Accessors for setting and discovering the iteration count of the Madgwick filter.
    */
    inline uint8_t madgwickIterations() {         return madgwick_iterations;  }
    inline void madgwickIterations(uint8_t nu) {
      if (nu < 10) madgwick_iterations = nu;
    }


    static float    mag_discard_threshold;
    static uint8_t  max_quats_per_event;   // Cut's down on overhead if load is high.
    static const char* getSourceTypeString(uint8_t);



  private:
    float delta_t      = 0.0f;

    //float GyroMeasError;
    float GyroMeasDrift;

    float offset_angle_y = 0.0f;
    float offset_angle_z = 0.0f;

    Vector3<float> _grav;                      // The Legend maintains an emirical value for gravity.

    PriorityQueue<InertialMeasurement*> quat_queue;   // This is the queue for quat operations.

    uint32_t irq_count_1 = 0;
    uint32_t irq_count_2 = 0;
    uint32_t irq_count_m = 0;

    uint32_t dirty_acc = 0;
    uint32_t dirty_gyr = 0;
    uint32_t dirty_mag = 0;

    LSM9DS1_AG* imu_ag = NULL;
    LSM9DS1_M*  imu_m = NULL;

    /* Pointers to our exported data. These should all point to a pool in the LegendManager
         that instantiated us. See that header file for more information. */
    Vector3<float>* _ptr_acc          = NULL;
    Vector3<float>* _ptr_gyr          = NULL;
    Vector3<float>* _ptr_mag          = NULL;
    Vector3<float>* _ptr_vel          = NULL;
    Vector3<float>* _ptr_null_grav    = NULL;   // Gravity-cancelled acceleration.
    Vector3<float>* _ptr_position     = NULL;    // Position of this IMU in space.
    Quaternion*     _ptr_quat         = NULL;
    float*          _ptr_temperature  = NULL;
    uint32_t*       _ptr_s_count_acc  = NULL;
    uint32_t*       _ptr_s_count_gyr  = NULL;
    uint32_t*       _ptr_s_count_mag  = NULL;
    uint32_t*       _ptr_s_count_temp = NULL;


    // A Legend might instruct us to handle our data in a certain way...
    uint32_t data_handling_flags = 0;

    //Vector3<float> gravity;        // If we need gravity, but the Legend doesn't want it.
    StringBuilder local_log;

    ManuvrRunnable quat_crunch_event;

    int8_t verbosity            =  1; // How chatty should this class be?
    int8_t pos_id               = -1; // We may find it convenient to lookup by sensor position.
    uint8_t madgwick_iterations =  1; //

    /* Is the class configured? Can we write our data to a pool?
    *  Used to trust the validity of our pointers.
    */
    inline bool legend_writable() {         return (data_handling_flags & IIU_DATA_HANDLING_LEGEND_WRITABLE);  }
    inline void legend_writable(bool en) {
      data_handling_flags = (en) ? (data_handling_flags | IIU_DATA_HANDLING_LEGEND_WRITABLE) : (data_handling_flags & ~(IIU_DATA_HANDLING_LEGEND_WRITABLE));
    }

    //// We probably want this fxn to return ms, and not s.
    //inline float findDeltaT(uint32_t now) {
    //  uint32_t op_ts = last_ts;
    //  last_ts  = now;
    //  return (now >= op_ts) ? (delta_t + ((now - op_ts) / 1000.0)) : (delta_t + (((0xFFFFFFFF - op_ts) - now) / 1000.0));
    //}

    // This is a privately-scoped override that does not consider the magnetometer.
    void MadgwickAHRSupdateIMU(InertialMeasurement*);
};


#endif
