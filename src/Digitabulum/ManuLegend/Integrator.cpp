/*
File:   Integrator.cpp
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

*/

#include "Integrator.h"
#include "ManuManager.h"

/*******************************************************************************
*      _______.___________.    ___   .___________. __    ______     _______.
*     /       |           |   /   \  |           ||  |  /      |   /       |
*    |   (----`---|  |----`  /  ^  \ `---|  |----`|  | |  ,----'  |   (----`
*     \   \       |  |      /  /_\  \    |  |     |  | |  |        \   \
* .----)   |      |  |     /  _____  \   |  |     |  | |  `----.----)   |
* |_______/       |__|    /__/     \__\  |__|     |__|  \______|_______/
*
* Static members and initializers should be located here.
*******************************************************************************/

static const Vector3<float> ZERO_VECTOR;
float    Integrator::mag_discard_threshold         = 0.8f;  // In Gauss.



const char* Integrator::getSourceTypeString(SampleType t) {
  switch (t) {
    case SampleType::ACCEL:          return "ACCEL";
    case SampleType::GYRO:           return "GYRO";
    case SampleType::MAG:            return "MAG";
    case SampleType::GRAVITY:        return "GRAVITY";
    case SampleType::BEARING:        return "BEARING";
    case SampleType::ALTITUDE:       return "ALTITUDE";
    case SampleType::LOCATION:       return "LOCATION";
    case SampleType::ALL:            return "ALL";
    case SampleType::UNSPECIFIED:    return "UNSPECIFIED";
    default:                         return "<UNDEFINED>";
  }
}


/*******************************************************************************
*   ___ _              ___      _ _              _      _
*  / __| |__ _ ______ | _ ) ___(_) |___ _ _ _ __| |__ _| |_ ___
* | (__| / _` (_-<_-< | _ \/ _ \ | / -_) '_| '_ \ / _` |  _/ -_)
*  \___|_\__,_/__/__/ |___/\___/_|_\___|_| | .__/_\__,_|\__\___|
*                                          |_|
* Constructors/destructors, class initialization functions and so-forth...
*******************************************************************************/

Integrator::Integrator() : _complete(CONFIG_INTEGRATOR_Q_DEPTH), _pending(CONFIG_INTEGRATOR_Q_DEPTH) {
  // Values for 9 DoF fusion and AHRS (Attitude and Heading Reference System)
  //GyroMeasError = 3.1415926535f * (40.0f / 180.0f);  // gyroscope measurement error in rads/s (shown as 3 deg/s)
  GyroMeasDrift = 3.1415926535f * (0.0f / 180.0f);   // gyroscope measurement drift in rad/s/s (shown as 0.0 deg/s/s)
  //beta = 0.866025404f * (3.1415926535f * GyroMeasError);   // compute beta
  beta = 0.2f;
}


Integrator::~Integrator() {
}



void Integrator::reset() {
  delta_t   = 0;
  dirty_mag = 0;
  dirty_acc = 0;
  dirty_gyr = 0;
  rangeBind(false);

  *(_ptr_temperature)    = 0.0f;
  *(_ptr_s_count_acc)     = 0;
  *(_ptr_s_count_gyr)     = 0;
  *(_ptr_s_count_mag)     = 0;
  *(_ptr_s_count_temp)    = 0;

  _grav.set(0.0f, 0.0f, 0.0f);
  if (_ptr_quat) _ptr_quat->set(0.0f, 0.0f, 0.0f, 1.0f);
  if (_ptr_acc)  _ptr_acc->set(0.0f, 0.0f, 0.0f);
  if (_ptr_gyr)  _ptr_gyr->set(0.0f, 0.0f, 0.0f);
  if (_ptr_mag)  _ptr_mag->set(0.0f, 0.0f, 0.0f);
  if (_ptr_vel)  _ptr_vel->set(0.0f, 0.0f, 0.0f);

  grav_scalar = 0.0f;
  offset_angle_y = 0.0f;
  offset_angle_z = 0.0f;
}


/*
TODO: DEPRECATED Slow. Too complicated.
*/
int8_t Integrator::pushMeasurement(SampleType data_type, float x, float y, float z, float d_t) {
  uint32_t read_time = micros();
  switch (data_type) {
    case SampleType::GYRO:
      if (_ptr_gyr) _ptr_gyr->set(x, y, z);
      //if (rangeBind()) {
      //  last_value_gyr /= LSM9DS1_M::max_range_vect_gyr;
      //}
      *(_ptr_s_count_gyr) = *(_ptr_s_count_gyr)+1;
      dirty_gyr = read_time;
      delta_t = (d_t > delta_t) ? d_t : delta_t;   // We want the smaller of the two, since it sets the quat rate.
      break;

    case SampleType::ACCEL:
      //if (_ptr_acc) _ptr_acc->set(x, y, z);
      if (_ptr_acc && !nullifyGravity()) _ptr_acc->set(x, y, z);  // If we are nulling gravity, can't do this.
      //if (rangeBind()) {
      //  last_value_acc /=  LSM9DS1_AG::max_range_vect_acc;
      //}
      *(_ptr_s_count_acc) = *(_ptr_s_count_acc)+1;
      dirty_acc = read_time;
      delta_t = (d_t > delta_t) ? d_t : delta_t;   // We want the smaller of the two, since it sets the quat rate.
      break;

    case SampleType::MAG:
      //if (rangeBind()) {
      //  last_value_mag /=  LSM9DS1_AG::max_range_vect_mag;
      //}
      if (correctSphericalAbberation()) {
        if (_ptr_mag) _ptr_mag->set(1.0f, 0.0f, 0.0f);
        return 0;
      }

      if (nullifyBearing()) {
        // Now we need to apply the rotation matrix so that we don't have to care what our bearing is at calibration time.
        // First, rotate about y.
        float sin_theta = sin((180.0f/M_PI) - offset_angle_y);
        float cos_theta = cos((180.0f/M_PI) - offset_angle_y);
        x = cos_theta * x + sin_theta * z;
        z = sin_theta * -1.0f * x + cos_theta * z;

        // Then rotate about z.
        sin_theta = sin((180.0f/M_PI) - offset_angle_z);
        cos_theta = cos((180.0f/M_PI) - offset_angle_z);
        x = cos_theta * x + sin_theta * -1.0f * y;
        y = sin_theta * x + cos_theta * y;
      }

      if (_ptr_mag) _ptr_mag->set(x, y, z);

      *(_ptr_s_count_mag) = *(_ptr_s_count_mag)+1;
      dirty_mag = read_time;
      break;

    case SampleType::GRAVITY:
      _grav.set(x, y, z);
      grav_scalar = _grav.length();
      _grav.normalize();
      // Now, we should reset the quaternion to reflect our idea of gravity with
      // whatever our last mag read was as the bearing.
      if (_ptr_quat) _ptr_quat->setDown(x, y, z);
      break;

    case SampleType::BEARING:
      if (grav_scalar != 0.0f) {  // We need the gravity vector to do this trick.
        Vector3<float> mag_original_bearing(x, y, z);
        mag_original_bearing.normalize();
        offset_angle_y = y;
        offset_angle_z = z;
        // Now, we should reset the quaternion to reflect our idea of gravity with
        // whatever our last mag read was as the bearing.

      }
      break;

    default:
      if (verbosity > 3) {
        local_log.concat("Integrator::pushMeasurement(): \t Unhandled Measurement type.\n");
        Kernel::log(&local_log);
      }
      break;
  }

  if (processQuats() && isQuatDirty()) {
    if (0 == _pending.count()) {
      // There was nothing in this queue, but there is about to be.
    }
    //SensorFrame* nu_measurement = fetchMeasurement();
    ////nu_measurement->set(_ptr_gyr, ((!dirty_mag && cleanMagZero()) ? (Vector3<float>*)&ZERO_VECTOR : _ptr_mag), _ptr_acc, delta_t);
    //if (dirty_mag) dirty_mag = 0;
    //dirty_acc = 0;
    //dirty_gyr = 0;
    //_pending.insert(nu_measurement);
  }

  return 0;
}

/**
* This is the function that the processing thread (if any) would call repeatedly.
*
* @return The number of SensorFrames completed.
*/
int8_t Integrator::churn() {
  int8_t return_value = 0;
  if ((0 < _complete.vacancy()) && MadgwickQuaternionUpdate()) {
    return_value++;
  }
  return return_value;
}



void Integrator::assign_legend_pointers(void* a, void* g, void* m,
                                 void* v, void* n_g, void* pos,
                                 void* q, void* t, void* sc_acc,
                                 void* sc_gyr, void* sc_mag, void* sc_temp)
{
  legend_writable(false);
  _ptr_acc          = (Vector3<float>*) a;
  _ptr_gyr          = (Vector3<float>*) g;
  _ptr_mag          = (Vector3<float>*) m;
  _ptr_vel          = (Vector3<float>*) v;
  _ptr_null_grav    = (Vector3<float>*) n_g;
  _ptr_position     = (Vector3<float>*) pos;
  _ptr_quat         = (Quaternion*)     q;
  _ptr_temperature  = (float*)          t;
  _ptr_s_count_acc  = (uint32_t*)       sc_acc;
  _ptr_s_count_gyr  = (uint32_t*)       sc_gyr;
  _ptr_s_count_mag  = (uint32_t*)       sc_mag;
  _ptr_s_count_temp = (uint32_t*)       sc_temp;

  if (q) {
    // If the Legend wants the quats, we turn them on.
    processQuats(true);
  }

  _ptr_quat->w = 1.0f;
  legend_writable(true);
}



bool Integrator::enableProfiling(bool en) {
  if (enableProfiling() != en) {
    data_handling_flags = (en) ? (data_handling_flags | IIU_DATA_HANDLING_PROFILING) : (data_handling_flags & ~(IIU_DATA_HANDLING_PROFILING));
  }
  return enableProfiling();
}


bool Integrator::nullGyroError(bool en) {
  if (nullGyroError() != en) {
    data_handling_flags = (en) ? (data_handling_flags | IIU_DATA_HANDLING_NULL_GYRO_ERROR) : (data_handling_flags & ~(IIU_DATA_HANDLING_NULL_GYRO_ERROR));
  }
  return nullGyroError();
}


bool Integrator::nullifyGravity(bool en) {
  if (nullifyGravity() != en) {
    data_handling_flags = (en) ? (data_handling_flags | IIU_DATA_HANDLING_NULLIFY_GRAVITY) : (data_handling_flags & ~(IIU_DATA_HANDLING_NULLIFY_GRAVITY));
    if (en && !processQuats()) {
      // This feature depends on quaternions.
      processQuats(true);
    }
    //imu_ag.cancel_error(en);
  }
  return nullifyGravity();
}


void Integrator::deposit_log(StringBuilder* _log) {
  local_log.concatHandoff(_log);
  Kernel::log(&local_log);
};


/**
* Debug support method. This fxn is only present in debug builds.
*
* @param   StringBuilder* The buffer into which this fxn should write its output.
*/
void Integrator::printDebug(StringBuilder* output) {
  output->concatf("\n-------------------------------------------------------\n--- Integrator\n-------------------------------------------------------\n-- %s legend\n-- Samples:\t%u\n", (legend_writable()?"writable":"invalid"), _frames_completed);
  output->concatf("-- Measurements\n\t_pending:   %u\n\t_complete:  %u\n", _pending.count(), _complete.count());
  output->concatf("-- delta_t:\t %3fms\n", ((double) delta_t * 1000));
  if (verbosity > 2) {
    if (verbosity > 3) output->concatf("-- GyroMeasDrift:    %.4f\n",  (double) GyroMeasDrift);
    output->concatf("-- Gravity: %s (%.4f, %.4f, %.4f)  %.4G\n", (nullifyGravity() ? "(nulled)":"        "), (double)(_grav.x), (double)(_grav.y), (double)(_grav.z), (double) (grav_scalar));
    output->concatf("-- offset_angle_y      %5.2f\n", (double) offset_angle_y);
    output->concatf("-- offset_angle_z      %5.2f\n", (double) offset_angle_z);
  }

  float grav_consensus = 0.0;
  for (uint8_t i = 0; i < 17; i++) {
    //grav_consensus += imus[i].grav_scalar;
  }
  grav_consensus /= 17;
  output->concatf("-- Gravity consensus:  %.4fg\n",  (double) grav_consensus);
  output->concat("\n");
}


void Integrator::dumpPointers(StringBuilder* output) {
  output->concatf("\t _ptr_quat         \t 0x%08x\n", (unsigned long) _ptr_quat        );
  output->concatf("\t _ptr_acc          \t 0x%08x\n", (unsigned long) _ptr_acc         );
  output->concatf("\t _ptr_gyr          \t 0x%08x\n", (unsigned long) _ptr_gyr         );
  output->concatf("\t _ptr_mag          \t 0x%08x\n", (unsigned long) _ptr_mag         );
  output->concatf("\t _ptr_temperature  \t 0x%08x\n", (unsigned long) _ptr_temperature );
  output->concatf("\t _ptr_vel          \t 0x%08x\n", (unsigned long) _ptr_vel         );
  output->concatf("\t _ptr_null_grav    \t 0x%08x\n", (unsigned long) _ptr_null_grav   );
  output->concatf("\t _ptr_position     \t 0x%08x\n", (unsigned long) _ptr_position    );
  output->concatf("\t _ptr_s_count_acc  \t 0x%08x\n", (unsigned long) _ptr_s_count_acc );
  output->concatf("\t _ptr_s_count_gyr  \t 0x%08x\n", (unsigned long) _ptr_s_count_gyr );
  output->concatf("\t _ptr_s_count_mag  \t 0x%08x\n", (unsigned long) _ptr_s_count_mag );
  output->concatf("\t _ptr_s_count_temp \t 0x%08x\n", (unsigned long) _ptr_s_count_temp);
}



/**
* Taken from
* https://github.com/kriswiner/LSM9DS1/blob/master/Teensy3.1/LSM9DS1-MS5637/quaternionFilters.ino
*/
// There is a tradeoff in the beta parameter between accuracy and response speed.
// In the original Madgwick study, beta of 0.041 (corresponding to GyroMeasError of 2.7 degrees/s) was found to give optimal accuracy.
// However, with this value, the LSM9SD0 response time is about 10 seconds to a stable initial quaternion.
// Subsequent changes also require a longish lag time to a stable output, not fast enough for a quadcopter or robot car!
// By increasing beta (GyroMeasError) by about a factor of fifteen, the response time constant is reduced to ~2 sec
// I haven't noticed any reduction in solution accuracy. This is essentially the I coefficient in a PID control sense;
// the bigger the feedback coefficient, the faster the solution converges, usually at the expense of accuracy.
// In any case, this is the free parameter in the Madgwick filtering and fusion scheme.
//const float Integrator::beta = ((float)sqrt(3.0f / 4.0f)) * Integrator::GyroMeasError;   // compute beta


/**
* This ought to be the only place where we promote vectors into the last_read position. Otherwise, there
*   shall be chaos as several different systems rely on that data member being synchronized WRT to the _ptr_quat->
*/
uint8_t Integrator::MadgwickQuaternionUpdate() {
  SensorFrame* c_frame = _pending.get();
  if (c_frame) {
    float d_t = c_frame->time();

    if (verbosity > 3) {
      local_log.concatf("At delta-t = %f: ", (double)d_t);
      #if defined(MANUVR_DEBUG)
        //c_frame->printDebug(&local_log);
        local_log.concat("\t");
        _ptr_quat->printDebug(&local_log);
      #endif
      local_log.concat("\n");
      Kernel::log(&local_log);
    }

    float q0 = _ptr_quat->w, q1= _ptr_quat->x, q2 = _ptr_quat->y, q3 = _ptr_quat->z;   // short name local variable for readability

    float norm;
    float s0, s1, s2, s3;
    float qDot1, qDot2, qDot3, qDot4;
    float hx, hy;
    float _2q0mx, _2q0my, _2q0mz, _2q1mx, _2bx, _2bz, _4bx, _4bz, _2q0, _2q1, _2q2, _2q3, q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;

    // Normalise mag c_frame
    float mag_normal;

    // Now we'll start the float churn...
    for (int set_i = 0; set_i < LEGEND_DATASET_IIU_COUNT; set_i++) {
      mag_normal = (c_frame->m_data[set_i]).normalize();

      if (dropObviousBadMag() && (mag_normal >= mag_discard_threshold)) {
        // We defer to the algorithm that does not use the non-earth mag data.
        for (int i = 0; i < madgwick_iterations; i++) {
          MadgwickAHRSupdateIMU(c_frame);
        }
      }
      else if (0.0f == mag_normal) {
        // We defer to the algorithm that does not use the absent mag data.
        for (int i = 0; i < madgwick_iterations; i++) {
          MadgwickAHRSupdateIMU(c_frame);
        }
      }
      else {
        float gx = (c_frame->g_data[set_i]).x * IIU_DEG_TO_RAD_SCALAR;
        float gy = (c_frame->g_data[set_i]).y * IIU_DEG_TO_RAD_SCALAR;
        float gz = (c_frame->g_data[set_i]).z * IIU_DEG_TO_RAD_SCALAR;

        for (int i = 0; i < madgwick_iterations; i++) {
          // Rate of change of quaternion from gyroscope
          qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
          qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
          qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
          qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

          // Normalise accelerometer c_frame. If vector is non-zero, integrate it...
          if (0.0f != (c_frame->a_data[set_i]).normalize()) {
            float mx = (c_frame->m_data[set_i]).x;
            float my = (c_frame->m_data[set_i]).y;
            float mz = (c_frame->m_data[set_i]).z;

            float ax = (c_frame->a_data[set_i]).x;
            float ay = (c_frame->a_data[set_i]).y;
            float az = (c_frame->a_data[set_i]).z;

            // Auxiliary variables to avoid repeated arithmetic
            _2q0mx = 2.0f * q0 * mx;
            _2q0my = 2.0f * q0 * my;
            _2q0mz = 2.0f * q0 * mz;
            _2q1mx = 2.0f * q1 * mx;
            _2q0 = 2.0f * q0;
            _2q1 = 2.0f * q1;
            _2q2 = 2.0f * q2;
            _2q3 = 2.0f * q3;
            q0q0 = q0 * q0;
            q0q1 = q0 * q1;
            q0q2 = q0 * q2;
            q0q3 = q0 * q3;
            q1q1 = q1 * q1;
            q1q2 = q1 * q2;
            q1q3 = q1 * q3;
            q2q2 = q2 * q2;
            q2q3 = q2 * q3;
            q3q3 = q3 * q3;

            // Reference direction of Earth's magnetic field
            hx = mx * q0q0 - _2q0my * q3 + _2q0mz * q2 + mx * q1q1 + _2q1 * my * q2 + _2q1 * mz * q3 - mx * q2q2 - mx * q3q3;
            hy = _2q0mx * q3 + my * q0q0 - _2q0mz * q1 + _2q1mx * q2 - my * q1q1 + my * q2q2 + _2q2 * mz * q3 - my * q3q3;
            _2bx = sqrt(hx * hx + hy * hy);
            _2bz = -_2q0mx * q2 + _2q0my * q1 + mz * q0q0 + _2q1mx * q3 - mz * q1q1 + _2q2 * my * q3 - mz * q2q2 + mz * q3q3;
            _4bx = 2.0f * _2bx;
            _4bz = 2.0f * _2bz;
            float _8bx = 2.0f * _4bx;
            float _8bz = 2.0f * _4bz;

            // Gradient decent algorithm corrective step
            s0 = -_2q2*(2*(q1q3 - q0q2) - ax) + _2q1*(2*(q0q1 + q2q3) - ay) +  -_4bz*q2*(_4bx*(0.5 - q2q2 - q3q3) + _4bz*(q1q3 - q0q2) - mx)   +   (-_4bx*q3+_4bz*q1)*(_4bx*(q1q2 - q0q3) + _4bz*(q0q1 + q2q3) - my)    +   _4bx*q2*(_4bx*(q0q2 + q1q3) + _4bz*(0.5 - q1q1 - q2q2) - mz);
            s1 = _2q3*(2*(q1q3 - q0q2) - ax)  + _2q0*(2*(q0q1 + q2q3) - ay) + -4*q1*(2*(0.5 - q1q1 - q2q2) - az)    +   _4bz*q3*(_4bx*(0.5 - q2q2 - q3q3) + _4bz*(q1q3 - q0q2) - mx)   + (_4bx*q2+_4bz*q0)*(_4bx*(q1q2 - q0q3) + _4bz*(q0q1 + q2q3) - my)   +   (_4bx*q3-_8bz*q1)*(_4bx*(q0q2 + q1q3) + _4bz*(0.5 - q1q1 - q2q2) - mz);
            s2 = -_2q0*(2*(q1q3 - q0q2) - ax) + _2q3*(2*(q0q1 + q2q3) - ay) + (-4*q2)*(2*(0.5 - q1q1 - q2q2) - az) +   (-_8bx*q2-_4bz*q0)*(_4bx*(0.5 - q2q2 - q3q3) + _4bz*(q1q3 - q0q2) - mx)+(_4bx*q1+_4bz*q3)*(_4bx*(q1q2 - q0q3) + _4bz*(q0q1 + q2q3) - my)+(_4bx*q0-_8bz*q2)*(_4bx*(q0q2 + q1q3) + _4bz*(0.5 - q1q1 - q2q2) - mz);
            s3 = _2q1*(2*(q1q3 - q0q2) - ax)  + _2q2*(2*(q0q1 + q2q3) - ay) + (-_8bx*q3+_4bz*q1)*(_4bx*(0.5 - q2q2 - q3q3) + _4bz*(q1q3 - q0q2) - mx)+(-_4bx*q0+_4bz*q2)*(_4bx*(q1q2 - q0q3) + _4bz*(q0q1 + q2q3) - my)+(_4bx*q1)*(_4bx*(q0q2 + q1q3) + _4bz*(0.5 - q1q1 - q2q2) - mz);

            norm = 1.0f / (float) sqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude

            // Apply feedback step
            qDot1 -= beta * (s0 * norm);
            qDot2 -= beta * (s1 * norm);
            qDot3 -= beta * (s2 * norm);
            qDot4 -= beta * (s3 * norm);

            // Integrate rate of change of quaternion to yield quaternion
            q0 += qDot1 * d_t;
            q1 += qDot2 * d_t;
            q2 += qDot3 * d_t;
            q3 += qDot4 * d_t;

            // Normalise quaternion
            norm = 1.0f / (float) sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q0 * q0);    // normalise quaternion
            _ptr_quat->w = q0 * norm;
            _ptr_quat->x = q1 * norm;
            _ptr_quat->y = q2 * norm;
            _ptr_quat->z = q3 * norm;
          }
        }
      }

      if (nullifyGravity()) {
        /* If we are going to cancel gravity, we should do so now. */
        _grav.x = (2 * (_ptr_quat->x * _ptr_quat->z - _ptr_quat->w * _ptr_quat->y));
        _grav.y = (2 * (_ptr_quat->w * _ptr_quat->x + _ptr_quat->y * _ptr_quat->z));
        _grav.z = (_ptr_quat->w * _ptr_quat->w - _ptr_quat->x * _ptr_quat->x - _ptr_quat->y * _ptr_quat->y + _ptr_quat->z * _ptr_quat->z);

        _ptr_null_grav->x = (c_frame->a_data[set_i]).x - _grav.x;
        _ptr_null_grav->y = (c_frame->a_data[set_i]).y - _grav.y;
        _ptr_null_grav->z = (c_frame->a_data[set_i]).z - _grav.z;

        if (findVelocity()) {
          // Are we finding velocity?
          _ptr_vel->x += _ptr_null_grav->x * d_t;
          _ptr_vel->y += _ptr_null_grav->y * d_t;
          _ptr_vel->z += _ptr_null_grav->z * d_t;

          if (trackPosition()) {
            // Track position....
            _ptr_position->x += _ptr_vel->x * d_t;
            _ptr_position->y += _ptr_vel->y * d_t;
            _ptr_position->z += _ptr_vel->z * d_t;
          }
        }
      }
    }
    c_frame->markComplete();
    if (_complete.insert(c_frame)) {
      local_log.concat("Dropped a frame in the integrator. This is probably a leak.\n");
    }
    _frames_completed++;
  }

  if (local_log.length() > 0) Kernel::log(&local_log);
  return 1;
}

//---------------------------------------------------------------------------------------------------
// IMU algorithm update

void Integrator::MadgwickAHRSupdateIMU(SensorFrame* c_frame) {
  float norm;
  float s0, s1, s2, s3;
  float qDot1, qDot2, qDot3, qDot4;
  float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 ,_8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

  float q0 = _ptr_quat->w, q1= _ptr_quat->x, q2 = _ptr_quat->y, q3 = _ptr_quat->z;   // short name local variable for readability

  for (int set_i = 0; set_i < LEGEND_DATASET_IIU_COUNT; set_i++) {
    float gx = (c_frame->g_data[set_i]).x * IIU_DEG_TO_RAD_SCALAR;
    float gy = (c_frame->g_data[set_i]).y * IIU_DEG_TO_RAD_SCALAR;
    float gz = (c_frame->g_data[set_i]).z * IIU_DEG_TO_RAD_SCALAR;
    float d_t = c_frame->time();

    // Rate of change of quaternion from gyroscope
    qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
    qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
    qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
    qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

    // Normalise accelerometer c_frame. If vector is non-zero, integrate it...
    if (0.0f != (c_frame->a_data[set_i]).normalize()) {
      float ax = (c_frame->a_data[set_i]).x;
      float ay = (c_frame->a_data[set_i]).y;
      float az = (c_frame->a_data[set_i]).z;
      // Auxiliary variables to avoid repeated arithmetic
      _2q0 = 2.0f * q0;
      _2q1 = 2.0f * q1;
      _2q2 = 2.0f * q2;
      _2q3 = 2.0f * q3;
      _4q0 = 4.0f * q0;
      _4q1 = 4.0f * q1;
      _4q2 = 4.0f * q2;
      _8q1 = 8.0f * q1;
      _8q2 = 8.0f * q2;
      q0q0 = q0 * q0;
      q1q1 = q1 * q1;
      q2q2 = q2 * q2;
      q3q3 = q3 * q3;

      // Gradient decent algorithm corrective step
      s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
      s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
      s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
      s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;

      norm = 1.0f / (float) sqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
      s0 *= norm;
      s1 *= norm;
      s2 *= norm;
      s3 *= norm;

      // Apply feedback step
      qDot1 -= beta * s0;
      qDot2 -= beta * s1;
      qDot3 -= beta * s2;
      qDot4 -= beta * s3;
    }

    // Integrate rate of change of quaternion to yield quaternion
    q0 += qDot1 * d_t;
    q1 += qDot2 * d_t;
    q2 += qDot3 * d_t;
    q3 += qDot4 * d_t;

    // Normalise quaternion
    norm = 1.0f / (float) sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q0 * q0);    // normalise quaternion
    _ptr_quat->w = q0 * norm;
    _ptr_quat->x = q1 * norm;
    _ptr_quat->y = q2 * norm;
    _ptr_quat->z = q3 * norm;
  }
}


int8_t Integrator::calibrate_from_data_ag() {
  //// Average vectors....
  //Vector3<int32_t> avg;
  //for (int i = 0; i < 32; i++) {
  //  avg.x += sample_backlog_acc[i].x;
  //  avg.y += sample_backlog_acc[i].y;
  //  avg.z += sample_backlog_acc[i].z;
  //}
  //avg /= 32;   // This is our idea of gravity.
  //noise_floor_acc.x = (int16_t) avg.x;
  //noise_floor_acc.y = (int16_t) avg.y;
  //noise_floor_acc.z = (int16_t) avg.z;

  //for (int i = 0; i < 32; i++) {
  //  avg.x += sample_backlog_gyr[i].x;
  //  avg.y += sample_backlog_gyr[i].y;
  //  avg.z += sample_backlog_gyr[i].z;
  //}
  //avg /= 32;   // This is our idea of gravity.
  //noise_floor_gyr.x = (int16_t) avg.x;
  //noise_floor_gyr.y = (int16_t) avg.y;
  //noise_floor_gyr.z = (int16_t) avg.z;

  //float scaler = error_map_acc[scale_acc].per_lsb;
  //float x = ((noise_floor_acc.x) * scaler);
  //float y = ((noise_floor_acc.y) * scaler);
  //float z = ((noise_floor_acc.z) * scaler);

  //// This is our idea of magentic North. Write it to the offset registers.
  ////for (int i = 0; i < 8; i++) {
  ////  *(register_pool + 14 + i) = *(register_pool + 3 + i);
  ////}
  ////writeRegister(RegID::AG_INT_THS_L_M, (register_pool + 12),   8, true);
  //scaler = error_map_gyr[scale_gyr].per_lsb;
  //Vector3<int16_t> reflection_vector_gyr(ManuManager::reflection_gyr.x, ManuManager::reflection_gyr.y, ManuManager::reflection_gyr.z);

  //x = ((int16_t) regValue(RegID::G_DATA_X)) * scaler * reflection_vector_gyr.x;
  //y = ((int16_t) regValue(RegID::G_DATA_Y)) * scaler * reflection_vector_gyr.y;
  //z = ((int16_t) regValue(RegID::G_DATA_Z)) * scaler * reflection_vector_gyr.z;
  ////Vector3<float> proj_xy(x, y, 0);
  ////Vector3<float> proj_xz(x, 0, z);
  ////Vector3<float> x_axis(1.0f, 0.0f, 0.0f);
  ////
  ////float offset_angle_y = Vector3<float>::angle(&proj_xy, &x_axis);
  ////float offset_angle_z = Vector3<float>::angle(&proj_xz, &x_axis);
  ////integrator->pushMeasurement(IMU_FLAG_BEARING_DATA, 0, offset_angle_y, offset_angle_z, 0);
  ////integrator->pushMeasurement(IMU_FLAG_BEARING_DATA, x, y, z, 0);
  return 0;
}



int8_t Integrator::calibrate_from_data_mag() {
  //// Is reading stable?
  //// Average vectors....
  //Vector3<int32_t> avg;
  //for (int i = 0; i < 32; i++) {
  //  avg.x += sample_backlog_mag[i].x;
  //  avg.y += sample_backlog_mag[i].y;
  //  avg.z += sample_backlog_mag[i].z;
  //}
  //avg /= 32;
  //noise_floor_mag.x = (int16_t) avg.x;
  //noise_floor_mag.y = (int16_t) avg.y;
  //noise_floor_mag.z = (int16_t) avg.z;
  return 0;
}
