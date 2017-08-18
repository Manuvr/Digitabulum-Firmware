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

Input Data
Orientation


*/

#ifndef __IIU_MEASUREMENT_H__
#define __IIU_MEASUREMENT_H__

#include <inttypes.h>
#include <DataStructures/Vector3.h>
#include <DataStructures/Quaternion.h>

#if defined(MANUVR_IMU_DEBUG)
#include <DataStructures/StringBuilder.h>
#endif  // MANUVR_IMU_DEBUG

/*
* Flags for SensorFrames
*/
#define  MANUFRAME_FLAGS_COMPLETE          0x01   // Is the data complete?


/*
* These are possible states for the frame.
*/
enum class FrameStage {
  /* ManuManager concerns. */
  IDLE     = 0x00,  // Freshly instanced (or wiped, if preallocated).
  INTAKE   = 0x01,  // Frame is waiting on data from the sensors.

  /* Integrator concerns. */
  ORI      = 0x02,  // Integrations for orientation.
  INTEG_0  = 0x03,  // Integrations depending on ORI (EG, gravity nullification)
  INTEG_1  = 0x04,  // Integrations depending on 2nd-degree integrations.
  ERR      = 0x05,  // Integrations for reporting error.

  /* These are finish states. */
  COMPLETE = 0x0F   // I/O op complete with no problems.
};


/*
* This is a container class that is pushed downstream to the Integrator class,
*   which will integrate it into a sensible representation of position and
*   orientation, taking error into account.
* Operations that span many elements are assumed to have element counts of 17.
*/
class SensorFrame {
  public:
    // TODO: The dynamic memory pool code that was in ManuLegend should be moved here.
    Vector4f         quat[17];  // Orientation
    Vector3<float> a_data[17];  // The vector of the accel data.
    Vector3<float> g_data[17];  // The vector of the gyro data.
    Vector3<float> m_data[17];  // The vector of the mag data.
    Vector3<float> n_data[17];  // Null-grav
    Vector3<float> v_data[17];  // Velocity
    Vector3<float> p_data[17];  // Position
    float     temperature[17];  // Temperature
    Vector3<float> hand_position;

    SensorFrame();

    void wipe();
    #if defined(MANUVR_IMU_DEBUG)
      void printDebug(StringBuilder* output);
    #endif

    inline uint32_t seq() {         return _seq;       };
    inline float   time() {         return _read_time; };
    inline void    time(float x) {  _read_time = x;    };

    inline void setO(uint8_t i, float w, float x, float y, float z) {
      quat[i].set(w, x, y, z);
    };

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

    inline FrameStage stage() {
      return _stage;
    };

    inline bool isComplete() {    return (FrameStage::COMPLETE == _stage);   };

    void markComplete() {
      _seq = ++SensorFrame::_total_sequences;
      _stage = FrameStage::COMPLETE;
    };


    static void resetSequenceCounter();



  private:
    uint32_t   _seq;        // Sequence number
    float      _read_time;  // Derived from the system time when the values arrived from the sensor.
    FrameStage _stage;      // Tracks the integration efforts across sync barriers.


    static uint32_t _total_sequences;  // We try to keep details hidden.
};

#endif  //__IIU_MEASUREMENT_H__
