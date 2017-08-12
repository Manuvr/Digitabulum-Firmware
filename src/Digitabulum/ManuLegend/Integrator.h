/*
File:   Integrator.h
Author: J. Ian Lindsay
Date:   2017.01.18

Copyright 2017 Manuvr, Inc

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.


The Integrator tracks measurements from an IMU and produces integal data such as
  velocity, position, and orientation.

This class was derived from the IIU class in Digitabulum r0, but has been
  stripped of its generality and connective role.

Integrator expects to be fed measured values of...
  1) Angular rate (via a gyro)
  2) Acceleration (via an accelerometer)
  3) Magnetic field vector
  4) Instrumentation-induced error, expressed as a relative value.

Error should be integrated here as well to form a set of limit error values for
  down-stream software. IE, datasets produced by this class ought to come with
  a quantitative measure of our confidence in it.

*/

#ifndef __INTEGRATOR_CLASS_H__
#define __INTEGRATOR_CLASS_H__

#include <Platform/Platform.h>
#include <DataStructures/Quaternion.h>
#include "SensorFrame.h"


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


#define IIU_STANDARD_GRAVITY     9.80665f // This is Earth's gravity at sea-level, in m/s^2
#define IIU_DEG_TO_RAD_SCALAR   (3.14159f / 180.0f)


#ifndef PREALLOCD_IMU_FRAMES
  #define PREALLOCD_IMU_FRAMES    10   // We retain this many frames.
#endif


enum class SampleType {
  UNSPECIFIED  = 0x00,
  ACCEL        = 0x01,  // Accelerometer vector.
  GYRO         = 0x02,  // Gyro vector.
  MAG          = 0x04,  // Magnetometer vector.
  GRAVITY      = 0x08,  // Gravity vector.
  BEARING      = 0x10,  // Bearing at calibration.
  ALTITUDE     = 0x20,  // Altitude.
  LOCATION     = 0x40,  // Location.
  ALL          = 0xFF  // All data available.
};



class Integrator {
  public:
    float beta;
    float grav_scalar = 0.0f;

    Integrator();
    ~Integrator();

    void dumpPointers(StringBuilder*);
    void printDebug(StringBuilder*);
    void setVerbosity(int8_t);

    int8_t init();

    /* Specific to this class */
    inline float getTemperature() {      return *(_ptr_temperature);  }
    void setTemperature(float);
    uint32_t totalSamples();

    void reset();

    /* These are meant to be called from the IMUs. */
    int8_t pushMeasurement(SampleType, float x, float y, float z, float delta_t);
    int8_t pushMeasurement(SensorFrame*);
    SensorFrame* takeResult();

    void deposit_log(StringBuilder*);

    /* These are meant to be called from a Legend. */
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

    inline bool isDirty() {         return (dirty_acc||dirty_gyr||dirty_mag); }
    inline bool isQuatDirty() {     return (dirty_acc & dirty_gyr);           }
    inline bool has_quats_left() {  return (frame_queue.size() > 0);           }


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


    /*
    * Accessors for setting and discovering the iteration count of the Madgwick filter.
    */
    inline uint8_t madgwickIterations() {         return madgwick_iterations;  }
    inline void madgwickIterations(uint8_t nu) {
      if (nu < 10) madgwick_iterations = nu;
    }

    /**
    * @return nullptr when empty.
    */
    inline SensorFrame* take() {  return _complete.get();  };


    static float    mag_discard_threshold;
    static const char* getSourceTypeString(SampleType);

    static SensorFrame* fetchMeasurement();



  private:
    RingBuffer<SensorFrame*> _complete(4);

    float delta_t      = 0.0f;

    //float GyroMeasError;
    float GyroMeasDrift;

    float offset_angle_y = 0.0f;
    float offset_angle_z = 0.0f;

    Vector3<float> _grav;   // The Integrator maintains an empirical value for gravity.

    uint32_t dirty_acc = 0;
    uint32_t dirty_gyr = 0;
    uint32_t dirty_mag = 0;

    /* Pointers to our exported data. These should all point to a pool in the ManuManager
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

    PriorityQueue<SensorFrame*> frame_queue;   // This is the queue for quat operations.

    int8_t verbosity            = 3;
    uint8_t madgwick_iterations = 1;

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
    void MadgwickAHRSupdateIMU(SensorFrame*);

    int8_t calibrate_from_data_mag();
    int8_t calibrate_from_data_ag();

    int8_t collect_reading_m();
    int8_t collect_reading_i();


    // Preallocated frames.
    // TODO: These things should not be static.
    static uint32_t prealloc_starves;
    static uint32_t measurement_heap_instantiated;
    static uint32_t measurement_heap_freed;
    static uint32_t minimum_prealloc_level;
    static PriorityQueue<SensorFrame*>  preallocd_measurements;
    static SensorFrame __prealloc[PREALLOCD_IMU_FRAMES];


    static void reclaimMeasurement(SensorFrame*);
};

#endif  // __INTEGRATOR_CLASS_H__
