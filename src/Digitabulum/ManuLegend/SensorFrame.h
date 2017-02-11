/*
File:   SensorFrame.h
Author: J. Ian Lindsay
Date:   2017.01.31

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


This is a container class for an IMU measurement frame.
*/

#ifndef __IIU_MEASUREMENT_H__
#define __IIU_MEASUREMENT_H__

#include <inttypes.h>
#include <DataStructures/Vector3.h>
#include <DataStructures/Quaternion.h>

#if defined(__IMU_DEBUG)
#include <DataStructures/StringBuilder.h>
#endif  // __IMU_DEBUG

/*
* This is a container class that is pushed downstream to the Integrator class,
*   which will integrate it into a sensible representation of position and
*   orientation, taking error into account.
* Operations that span many elements are assumed to have element counts of 17.
*/
class SensorFrame {
  public:
    Vector3<float> g_data[17];  // The vector of the gyro data.
    Vector3<float> a_data[17];  // The vector of the accel data.
    Vector3<float> m_data[17];  // The vector of the mag data.
    Vector3<float> v_data[17];  // Velocity
    Vector3<float> n_data[17];  // Null-grav
    Vector3<float> p_data[17];  // Position
    Vector4f         quat[17];    // Orientation

    SensorFrame();

    void wipe();
    #if defined(__IMU_DEBUG)
      void printDebug(StringBuilder* output);
    #endif

    inline float time() {          return read_time;  };
    inline void  time(float nu) {  read_time = nu;    };

    inline void setI(uint8_t i, float ax, float ay, float az, float gx, float gy, float gz) {
      a_data[i](ax, ay, az);
      g_data[i](gx, gy, gz);
    };

    inline void setM(uint8_t i, float x, float y, float z) {
      m_data[i](x, y, z);
    };

    inline void setV(uint8_t i, float x, float y, float z) {
      v_data[i](x, y, z);
    };

    inline void setN(uint8_t i, float x, float y, float z) {
      n_data[i](x, y, z);
    };

    inline void setP(uint8_t i, float x, float y, float z) {
      p_data[i](x, y, z);
    };


  private:
    uint32_t seq;          // Sequence number
    float    temperature;  //
    float    read_time;    // Derived from the system time when the values arrived from the sensor.
};

#endif  //__IIU_MEASUREMENT_H__
