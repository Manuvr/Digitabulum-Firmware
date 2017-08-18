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
  5) Temperature

These values need to be float and pre-filtered according to the specs of specific IMUs.
  Fortunately, it's cheaper to do it there anyhow while the data is still integer-wide.
  ARM DSP instructions will not be useful in this class. All those concerns should stop
  at ManuManager,

Integrator produces:
  6) Orientation, expressed as a quaternion.
  7) Gravity-canceled acceleration
  8) Velocity
  9) Position

Error should be integrated here as well to form a set of limit error values for
  down-stream software. IE, datasets produced by this class ought to come with
  a quantitative measure of our confidence in it.

*/

#ifndef __INTEGRATOR_CLASS_H__
#define __INTEGRATOR_CLASS_H__

#include <Platform/Platform.h>
#include <DataStructures/Quaternion.h>
#include <DataStructures/RingBuffer.h>
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

// This is Earth's gravity at sea-level, in m/s^2
#define IIU_STANDARD_GRAVITY     9.80665f
#define IIU_DEG_TO_RAD_SCALAR   (3.14159f / 180.0f)


enum class SampleType {
  UNSPECIFIED  = 0x00,
  ACCEL        = 0x01,  // Accelerometer vector.
  GYRO         = 0x02,  // Gyro vector.
  MAG          = 0x04,  // Magnetometer vector.
  GRAVITY      = 0x08,  // Gravity vector.
  BEARING      = 0x10,  // Bearing at calibration.
  ALTITUDE     = 0x20,  // Altitude.
  LOCATION     = 0x40,  // Location.
  ALL          = 0xFF   // All data available.
};


class Integrator {
  public:
    float beta;
    float grav_scalar = 0.0f;

    Integrator();
    ~Integrator();

    void printDebug(StringBuilder*);
    inline void setVerbosity(int8_t nu) {   verbosity = nu;   };


    int8_t init();
    void reset();

    /**
    * @return How many frames the integrator has processed.
    */
    inline uint32_t totalFrames() {  return _frames_completed;  };

    /**
    * @param SensorFrame* The frame to be integrated.
    * @return non-zero on error.
    */
    inline int8_t pushFrame(SensorFrame* x) {  return _pending.insert(x);  };
    /**
    * @return nullptr when empty.
    */
    inline SensorFrame* takeResult() {         return _complete.get();        };
    inline unsigned int resultsWaiting() {     return _complete.count();      };
    inline bool         has_quats_left() {     return (_pending.count() > 0); };
    int8_t churn();


    /*
    * Accessors for profiling.
    * TODO: Inline these
    */
    inline bool enableProfiling() {        return (data_handling_flags & IIU_DATA_HANDLING_PROFILING);  }
    bool enableProfiling(bool en);

    /*
    * Accessors for gravity cancelation.
    */
    inline bool nullifyGravity() {         return (data_handling_flags & IIU_DATA_HANDLING_NULLIFY_GRAVITY);  }
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


    static float    mag_discard_threshold;



  private:
    // Queues for cycling frames.
    RingBuffer<SensorFrame*> _complete;
    RingBuffer<SensorFrame*> _pending;
    StringBuilder local_log;

    float delta_t      = 0.0f;

    //float GyroMeasError;
    float GyroMeasDrift;

    Vector3<float> _grav;   // The Integrator maintains an empirical value for gravity.

    // A Legend might instruct us to handle our data in a certain way...
    uint32_t data_handling_flags = 0;
    uint32_t _frames_completed   = 0;    // Profiling member.
    int8_t   verbosity           = 3;    //
    uint8_t  madgwick_iterations = 1;    // Madgwick's filter is run this many times per frame.


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

    uint8_t MadgwickQuaternionUpdate();
    // This is a privately-scoped override that does not consider the magnetometer.
    void MadgwickAHRSupdateIMU(SensorFrame*);

    int8_t calibrate_from_data_mag();
    int8_t calibrate_from_data_ag();

    int8_t collect_reading_m();
    int8_t collect_reading_i();
};

#endif  // __INTEGRATOR_CLASS_H__
