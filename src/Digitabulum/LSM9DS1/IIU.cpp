/*
File:   IIU.cpp
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


#include "IIU.h"
#include <stdarg.h>

#include "ManuLegend/ManuManager.h"


const Vector3<float> ZERO_VECTOR;

/****************************************************************************************************
*      _______.___________.    ___   .___________. __    ______     _______.
*     /       |           |   /   \  |           ||  |  /      |   /       |
*    |   (----`---|  |----`  /  ^  \ `---|  |----`|  | |  ,----'  |   (----`
*     \   \       |  |      /  /_\  \    |  |     |  | |  |        \   \
* .----)   |      |  |     /  _____  \   |  |     |  | |  `----.----)   |
* |_______/       |__|    /__/     \__\  |__|     |__|  \______|_______/
*
* Static members and initializers should be located here. Initializers first, functions second.
****************************************************************************************************/
uint8_t  IIU::max_quats_per_event    = 2;
float    IIU::mag_discard_threshold  = 0.8f;  // In Gauss.

/****************************************************************************************************
* Class-management functions...                                                                     *
****************************************************************************************************/

IIU::IIU() {
}


IIU::~IIU() {
  while (quat_queue.hasNext()) {
    // We need to return the preallocated measurements we are holding.
    LegendManager::reclaimMeasurement(quat_queue.dequeue());
  }
}

void IIU::class_init(uint8_t idx) {
  pos_id = idx;
  imu_ag.class_init(CPLD_REG_IMU_DM_P_I + idx, this);    // Inertial aspect.
  imu_m.class_init(CPLD_REG_IMU_DM_P_M + idx, this);     // Magnetic aspect.
  //TODO: Refugees from setPositionAndAddress(uint8_t nu_pos).
  if (quat_crunch_event.argCount() > 0) {
    quat_crunch_event.clearArgs();
  }

  // Values for 9 DoF fusion and AHRS (Attitude and Heading Reference System)
  //GyroMeasError = 3.1415926535f * (40.0f / 180.0f);  // gyroscope measurement error in rads/s (shown as 3 deg/s)
  GyroMeasDrift = 3.1415926535f * (0.0f / 180.0f);   // gyroscope measurement drift in rad/s/s (shown as 0.0 deg/s/s)
  //beta = 0.866025404f * (3.1415926535f * GyroMeasError);   // compute beta
  beta = 0.2f;

  /* Setup our pre-formed quat crunch event. */
  quat_crunch_event.repurpose(DIGITABULUM_MSG_IMU_QUAT_CRUNCH, (EventReceiver*) LegendManager::getInstance());
  quat_crunch_event.specific_target = (EventReceiver*) LegendManager::getInstance();
  quat_crunch_event.incRefs();
  //quat_crunch_event.priority(4);
  quat_crunch_event.addArg((uint8_t) pos_id);

  //TODO: End of Refugees from setPositionAndAddress(uint8_t nu_pos).
}


/*
* This init() function will call all downstream init() functions. Because of that fact, we need to know a priori
*   which logical sensors are in the same physical package, and they need to have been passed in by now. Note that
*   this does not apply to EC devs, as those are typically added as an indirect result of this call.
*/
int8_t IIU::init() {
  int8_t return_value = 0;

  if (!imu_ag.initComplete()) {
    return_value = -1;
  }
  else if (!imu_m.initComplete()) {
    return_value = -1;
  }

  if (! imu_m.present()) {
    imu_m.setDesiredState(State::STAGE_1);
  }

  if (! imu_ag.present()) {
    imu_ag.setDesiredState(State::STAGE_1);
  }
  return return_value;
}


void IIU::reset() {
  imu_m.reset();
  imu_ag.reset();
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

  irq_count_1 = 0;
  irq_count_2 = 0;
  irq_count_m = 0;
}


void IIU::sync() {
  imu_ag.bulk_refresh();
  imu_m.bulk_refresh();
}


void IIU::setOperatingState(uint8_t nu) {
  imu_m.setDesiredState(LSM9DSx_Common::getStateByIndex(nu));
  imu_ag.setDesiredState(LSM9DSx_Common::getStateByIndex(nu));
}


uint32_t IIU::totalSamples() {
  return *(_ptr_s_count_gyr) + *(_ptr_s_count_acc) + *(_ptr_s_count_mag);
}


/*
*/
int8_t IIU::pushMeasurement(uint8_t data_type, float x, float y, float z, float d_t) {
  uint32_t read_time = micros();
  switch (data_type) {
    case IMU_FLAG_GYRO_DATA:
      if (_ptr_gyr) _ptr_gyr->set(x, y, z);
      //if (rangeBind()) {
      //  last_value_gyr /= LSM9DS1_M::max_range_vect_gyr;
      //}
      *(_ptr_s_count_gyr) = *(_ptr_s_count_gyr)+1;
      dirty_gyr = read_time;
      delta_t = (d_t > delta_t) ? d_t : delta_t;   // We want the smaller of the two, since it sets the quat rate.
      break;

    case IMU_FLAG_ACCEL_DATA:
      //if (_ptr_acc) _ptr_acc->set(x, y, z);
      if (_ptr_acc && !nullifyGravity()) _ptr_acc->set(x, y, z);  // If we are nulling gravity, can't do this.
      //if (rangeBind()) {
      //  last_value_acc /=  LSM9DS1_AG::max_range_vect_acc;
      //}
      *(_ptr_s_count_acc) = *(_ptr_s_count_acc)+1;
      dirty_acc = read_time;
      delta_t = (d_t > delta_t) ? d_t : delta_t;   // We want the smaller of the two, since it sets the quat rate.
      break;

    case IMU_FLAG_MAG_DATA:
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

    case IMU_FLAG_GRAVITY_DATA:
      _grav.set(x, y, z);
      grav_scalar = _grav.length();
      _grav.normalize();
      // Now, we should reset the quaternion to reflect our idea of gravity with
      // whatever our last mag read was as the bearing.
      if (_ptr_quat) _ptr_quat->setDown(x, y, z);
      break;

    case IMU_FLAG_BEARING_DATA:
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
        local_log.concatf("%d\t IIU::pushMeasurement(): \t Unhandled Measurement type.\n", pos_id);
        Kernel::log(&local_log);
      }
      break;
  }

  if (processQuats() && isQuatDirty()) {
    if (0 == quat_queue.size()) {
      // There was nothing in this queue, but there is about to be.
      Kernel::staticRaiseEvent(&quat_crunch_event);
    }
    InertialMeasurement* nu_measurement = LegendManager::fetchMeasurement(data_type);
    nu_measurement->set(_ptr_gyr, ((!dirty_mag && cleanMagZero()) ? (Vector3<float>*)&ZERO_VECTOR : _ptr_mag), _ptr_acc, delta_t);
    if (dirty_mag) dirty_mag = 0;
    dirty_acc = 0;
    dirty_gyr = 0;

    quat_queue.insert(nu_measurement, 0xFFFFFFFF - read_time);
  }

  return 0;
}


int8_t IIU::state_change_notice(LSM9DSx_Common* imu_ptr, State p_state, State c_state) {
  switch (c_state) {
    case State::STAGE_3:
      if (_ptr_vel)  _ptr_vel->set(0.0f, 0.0f, 0.0f);
      if (_ptr_quat) _ptr_quat->set(0.0f, 0.0f, 0.0f, 1.0f);
      break;
    case State::UNDEF:
      // Mistake.
      break;
    default:
      // No action.
      break;
  }
  return 0;
}



void IIU::assign_legend_pointers(void* a, void* g, void* m,
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


void IIU::assign_register_pointers(IMURegisterPointers* _reg) {
  for (size_t i = 0; i < sizeof(IMURegisterPointers); i+=4) {
    *(&_reg_ptrs + i) = *(_reg + i);
  }
}


/*
* For now:
* 0: Off  (Implies other register activity)
* 1: Lowest rate while still collecting data.
* 2: Low-accuracy rate. ~100Hz from each FIFO'd sensor.
* 3: Moderate rate.
* 4: Highest rate supported.
*/
void IIU::setSampleRateProfile(uint8_t profile_idx) {
  switch (profile_idx) {
    case 0:
      delta_t = 0.0f;
      imu_ag.set_sample_rate_acc(0);
      imu_ag.set_sample_rate_gyr(0);
      imu_m.set_sample_rate_mag(0);
      break;
    case 1:
      delta_t = 0.0f;
      imu_ag.set_sample_rate_acc(1);
      imu_ag.set_sample_rate_gyr(1);
      imu_m.set_sample_rate_mag(1);
      break;
    case 2:
      delta_t = 0.0f;
      imu_ag.set_sample_rate_acc(6);
      imu_ag.set_sample_rate_gyr(2);
      imu_m.set_sample_rate_mag(5);
      break;
    case 3:
      delta_t = 0.0f;
      imu_ag.set_sample_rate_acc(8);
      imu_ag.set_sample_rate_gyr(3);
      imu_m.set_sample_rate_mag(5);
      break;
    case 4:
      delta_t = 0.0f;
      imu_ag.set_sample_rate_acc(10);
      imu_ag.set_sample_rate_gyr(4);
      imu_m.set_sample_rate_mag(6);
      break;
    default:
      break;
  }
}


int8_t IIU::setParameter(uint16_t reg, int len, uint8_t *data) {
  switch (reg) {
    case IIU_PARAM_FRAME_QUEUE_LENGTH:
      return 0;
  }
  return -1;
}


int8_t IIU::getParameter(uint16_t reg, int len, uint8_t*) {
  return -1;
}



bool IIU::state_pass_through(uint8_t nu) {
  imu_m.setDesiredState(LSM9DSx_Common::getStateByIndex(nu));
  imu_ag.setDesiredState(LSM9DSx_Common::getStateByIndex(nu));

  return (imu_m.desired_state_attained() & imu_ag.desired_state_attained());
}



int8_t IIU::readSensor(void) {
  if (State::STAGE_3 <= imu_m.getState()) {
    imu_m.setDesiredState(State::STAGE_5);
    imu_m.readSensor();
  }

  if (State::STAGE_3 <= imu_ag.getState()) {
    imu_ag.setDesiredState(State::STAGE_5);
    imu_ag.readSensor();
  }
  return 0;
}


bool IIU::enableProfiling(bool en) {
  if (enableProfiling() != en) {
    data_handling_flags = (en) ? (data_handling_flags | IIU_DATA_HANDLING_PROFILING) : (data_handling_flags & ~(IIU_DATA_HANDLING_PROFILING));
    imu_m.profile(en);
    imu_ag.profile(en);
  }
  return enableProfiling();
}


bool IIU::nullGyroError(bool en) {
  if (nullGyroError() != en) {
    data_handling_flags = (en) ? (data_handling_flags | IIU_DATA_HANDLING_NULL_GYRO_ERROR) : (data_handling_flags & ~(IIU_DATA_HANDLING_NULL_GYRO_ERROR));
    imu_m.cancel_error(en);
  }
  return nullGyroError();
}


bool IIU::nullifyGravity(bool en) {
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



void IIU::deposit_log(StringBuilder* _log) {
  local_log.concatHandoff(_log);
  Kernel::log(&local_log);
};




/*
* TODO: These are on borrowed time. I am likely to inline them.
*/
    int8_t IIU::irq_ag0() {
      irq_count_2++;
      if (verbosity > 4) {
        local_log.concatf("IIU %d \t irq_ag0\n", pos_id);
        Kernel::log(&local_log);
      }
      /* Hard-coding the tap to the secondary IRQ pin for now. */
      ManuvrMsg* event = Kernel::returnEvent(DIGITABULUM_MSG_IMU_TAP);
      event->addArg((uint8_t) pos_id);  // We must cast this, because it is *really* an int8.
      event->setOriginator(LegendManager::getInstance());
      Kernel::staticRaiseEvent(event);
      //return imu_ag.irq_0();
      return 0;
    }

    int8_t IIU::irq_ag1() {
      irq_count_1++;
      if (verbosity > 4) {
        local_log.concatf("IIU %d \t irq_ag1\n", pos_id);
        Kernel::log(&local_log);
      }
      return imu_ag.irq_1();
    }

    int8_t IIU::irq_m() {
      irq_count_m++;
      if (verbosity > 4) {
        local_log.concatf("IIU %d \t irq_m\n", pos_id);
        Kernel::log(&local_log);
      }
      return imu_m.irq();
    }




void IIU::setTemperature(float nu) {
  if (_ptr_temperature)  *(_ptr_temperature) = nu;
  if (_ptr_s_count_temp) *(_ptr_s_count_temp) = *(_ptr_s_count_temp)+1;
}


void IIU::enableAutoscale(uint8_t s_type, bool enabled) {
  if (s_type & IMU_FLAG_GYRO_DATA) {
    imu_ag.autoscale_gyr(enabled);
  }
  if (s_type & IMU_FLAG_MAG_DATA) {
    imu_m.autoscale_mag(enabled);
  }
  if (s_type & IMU_FLAG_ACCEL_DATA) {
    imu_ag.autoscale_acc(enabled);
  }
}



void IIU::setVerbosity(int8_t nu) {
  imu_m.setVerbosity(nu);
  imu_ag.setVerbosity(nu);
  verbosity = nu;
}


void IIU::printLastFrame(StringBuilder *output) {
  if (nullptr == output) return;
  output->concatf("--- (MAG) (%.4f, %.4f, %.4f)",  (double)(_ptr_mag->x), (double)(_ptr_mag->y), (double)(_ptr_mag->z));
  output->concatf("\t(ACCEL) (%.4f, %.4f, %.4f)",  (double)(_ptr_acc->x), (double)(_ptr_acc->y), (double)(_ptr_acc->z));
  output->concatf("\t(GYRO) (%.4f, %.4f, %.4f)\n", (double)(_ptr_gyr->x), (double)(_ptr_gyr->y), (double)(_ptr_gyr->z));
}


/**
* Debug support method. This fxn is only present in debug builds.
*
* @param   StringBuilder* The buffer into which this fxn should write its output.
*/
void IIU::printDebug(StringBuilder* output) {
  if (nullptr == output) return;
  output->concatf("\n-------------------------------------------------------\n--- IIU %d\n-------------------------------------------------------\n--- %s legend\n--- Samples:\t ", pos_id, (legend_writable()?"writable":"invalid"));
  printBrief(output);  // OK
  output->concatf("--- measurements\n--- quat_queue:\t %d measurements\n", quat_queue.size());
  output->concatf("--- temperature: %.2fC\n--- delta_t:\t %.4fms\n--- Quat:\t ", ((double) delta_t * 1000), (double)*(_ptr_temperature));
  #if defined(__MANUVR_DEBUG)
    _ptr_quat->printDebug(output);  // OK
  #endif
  output->concat("\n");
  if (verbosity > 2) {
    if (verbosity > 3) output->concatf("--- GyroMeasDrift:    %.4f\n",  (double) GyroMeasDrift);
    output->concatf("--- Gravity: %s (%.4f, %.4f, %.4f)  %.4G\n", (nullifyGravity() ? "(nulled)":"        "), (double)(_grav.x), (double)(_grav.y), (double)(_grav.z), (double) (grav_scalar));
    output->concatf("--- offset_angle_y      %5.2f\n", (double) offset_angle_y);
    output->concatf("--- offset_angle_z      %5.2f\n", (double) offset_angle_z);
    printLastFrame(output);
  }

  output->concatf("--- IRQ hits: \t xm_1: %u \t xm_2: %u \t gyr_1: %u \n", irq_count_1, irq_count_2, irq_count_m);
  if (verbosity > 0) {
    imu_m.dumpDevRegs(output);
    output->concat("\n");
    imu_ag.dumpDevRegs(output);
  }
  output->concat("\n\n");
}


void IIU::printBrief(StringBuilder* output) {
  if (imu_m.initComplete() && imu_ag.initComplete()) {
    output->concatf("%8u acc  %8u gyr  %8u mag  %8u temp\n", (unsigned long) *(_ptr_s_count_acc), (unsigned long) *(_ptr_s_count_gyr), (unsigned long) *(_ptr_s_count_mag), (unsigned long) *(_ptr_s_count_temp));
  }
  else {
    output->concatf("XM state: %s  \t  G state: %s\n", imu_ag.getStateString(), imu_m.getStateString());
  }
}


void IIU::dumpPreformedElements(StringBuilder* output) {
  if (nullptr == output) return;
  output->concat("\n-------------------------------------------------------\n--- IIU \n-------------------------------------------------------\n");
  output->concat("--- Quat-crunch event\n");
  #if defined(__MANUVR_DEBUG)
    quat_crunch_event.printDebug(output);
  #endif

  imu_m.dumpPreformedElements(output);
  output->concat("\n");
  imu_ag.dumpPreformedElements(output);
  output->concat("\n\n");
}


void IIU::dumpPointers(StringBuilder* output) {
  output->concatf("\t _ptr_quat         \t 0x%08x\n",    (unsigned long)  _ptr_quat        );
  output->concatf("\t _ptr_acc          \t 0x%08x\n",    (unsigned long)  _ptr_acc         );
  output->concatf("\t _ptr_gyr          \t 0x%08x\n",    (unsigned long)  _ptr_gyr         );
  output->concatf("\t _ptr_mag          \t 0x%08x\n",    (unsigned long)  _ptr_mag         );
  output->concatf("\t _ptr_temperature  \t 0x%08x\n",    (unsigned long)  _ptr_temperature );
  output->concatf("\t _ptr_vel          \t 0x%08x\n",    (unsigned long)  _ptr_vel         );
  output->concatf("\t _ptr_null_grav    \t 0x%08x\n",    (unsigned long)  _ptr_null_grav   );
  output->concatf("\t _ptr_position     \t 0x%08x\n",    (unsigned long)  _ptr_position    );
  output->concatf("\t _ptr_s_count_acc  \t 0x%08x\n",    (unsigned long)  _ptr_s_count_acc );
  output->concatf("\t _ptr_s_count_gyr  \t 0x%08x\n",    (unsigned long)  _ptr_s_count_gyr );
  output->concatf("\t _ptr_s_count_mag  \t 0x%08x\n",    (unsigned long)  _ptr_s_count_mag );
  output->concatf("\t _ptr_s_count_temp \t 0x%08x\n",    (unsigned long)  _ptr_s_count_temp);
}


void IIU::dumpRegisterPointers(StringBuilder* output) {
}



const char* IIU::getSourceTypeString(uint8_t t) {
  switch (t) {
    case IMU_FLAG_ACCEL_DATA:          return "ACCEL_DATA";
    case IMU_FLAG_GYRO_DATA:           return "GYRO_DATA";
    case IMU_FLAG_MAG_DATA:            return "MAG_DATA";
    case IMU_FLAG_GRAVITY_DATA:        return "GRAVITY_DATA";
    case IMU_FLAG_UNSPECIFIED_DATA:    return "UNSPECIFIED";
    default:                           return "<UNDEFINED>";
  }
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
//const float IIU::beta = ((float)sqrt(3.0f / 4.0f)) * IIU::GyroMeasError;   // compute beta


/**
* This ought to be the only place where we promote vectors into the last_read position. Otherwise, there
*   shall be chaos as several different systems rely on that data member being synchronized WRT to the _ptr_quat->
*/
uint8_t IIU::MadgwickQuaternionUpdate() {
  uint8_t quats_procd_this_run = 0;

  InertialMeasurement* measurement;

  while ((quats_procd_this_run < max_quats_per_event) && quat_queue.hasNext()) {
    measurement = quat_queue.dequeue();
    float d_t = measurement->read_time;

    if (verbosity > 3) {
      local_log.concatf("At delta-t = %f: ", (double)d_t);
      #if defined(__MANUVR_DEBUG)
        measurement->printDebug(&local_log);
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

    // Normalise mag measurement
    float mag_normal = (measurement->m_data).normalize();

    if (dropObviousBadMag() && (mag_normal >= mag_discard_threshold)) {
      // We defer to the algorithm that does not use the non-earth mag data.
      for (int i = 0; i < madgwick_iterations; i++) {
        MadgwickAHRSupdateIMU(measurement);
      }
    }
    else if (0.0f == mag_normal) {
      // We defer to the algorithm that does not use the absent mag data.
      for (int i = 0; i < madgwick_iterations; i++) {
        MadgwickAHRSupdateIMU(measurement);
      }
    }
    else {
      float gx = (measurement->g_data).x * IIU_DEG_TO_RAD_SCALAR;
      float gy = (measurement->g_data).y * IIU_DEG_TO_RAD_SCALAR;
      float gz = (measurement->g_data).z * IIU_DEG_TO_RAD_SCALAR;

      for (int i = 0; i < madgwick_iterations; i++) {
        // Rate of change of quaternion from gyroscope
        qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
        qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
        qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
        qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

        // Normalise accelerometer measurement. If vector is non-zero, integrate it...
        if (0.0f != (measurement->a_data).normalize()) {
          float mx = (measurement->m_data).x;
          float my = (measurement->m_data).y;
          float mz = (measurement->m_data).z;

          float ax = (measurement->a_data).x;
          float ay = (measurement->a_data).y;
          float az = (measurement->a_data).z;

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

      _ptr_null_grav->x = (measurement->a_data).x - _grav.x;
      _ptr_null_grav->y = (measurement->a_data).y - _grav.y;
      _ptr_null_grav->z = (measurement->a_data).z - _grav.z;

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

    quats_procd_this_run++;          // Bailout still counts as proc.
    LegendManager::reclaimMeasurement(measurement); // Release the memory. Even on bailout.
  }

  if (local_log.length() > 0) Kernel::log(&local_log);
  return quats_procd_this_run;
}

//---------------------------------------------------------------------------------------------------
// IMU algorithm update

void IIU::MadgwickAHRSupdateIMU(InertialMeasurement* measurement) {
  float norm;
  float s0, s1, s2, s3;
  float qDot1, qDot2, qDot3, qDot4;
  float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 ,_8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

  float q0 = _ptr_quat->w, q1= _ptr_quat->x, q2 = _ptr_quat->y, q3 = _ptr_quat->z;   // short name local variable for readability

  float gx = (measurement->g_data).x * IIU_DEG_TO_RAD_SCALAR;
  float gy = (measurement->g_data).y * IIU_DEG_TO_RAD_SCALAR;
  float gz = (measurement->g_data).z * IIU_DEG_TO_RAD_SCALAR;
  float d_t = measurement->read_time;

  // Rate of change of quaternion from gyroscope
  qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
  qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
  qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
  qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

  // Normalise accelerometer measurement. If vector is non-zero, integrate it...
  if (0.0f != (measurement->a_data).normalize()) {
    float ax = (measurement->a_data).x;
    float ay = (measurement->a_data).y;
    float az = (measurement->a_data).z;
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
