/*
File:   LegendManager.cpp
Author: J. Ian Lindsay
Date:   2014.07.01

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

#include "ManuLegend.h"

extern "C" {
  extern volatile CPLDDriver* cpld;
}



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

Vector3<int16_t> LegendManager::reflection_mag;
Vector3<int16_t> LegendManager::reflection_acc;
Vector3<int16_t> LegendManager::reflection_gyr;

LegendManager* LegendManager::INSTANCE = NULL;

LegendManager* LegendManager::getInstance() {
  return INSTANCE;
}

uint32_t LegendManager::measurement_heap_instantiated = 0;
uint32_t LegendManager::measurement_heap_freed        = 0;

uint32_t LegendManager::prealloc_starves = 0;

PriorityQueue<InertialMeasurement*>  LegendManager::preallocd_measurements;

uint32_t LegendManager::minimum_prealloc_level = PREALLOCATED_IIU_MEASUREMENTS;



InertialMeasurement* LegendManager::fetchMeasurement(uint8_t type_code) {
  InertialMeasurement* return_value;

  if (0 == preallocd_measurements.size()) {
    // We have exhausted our preallocated measurements. Note it.
    prealloc_starves++;
    return_value = new InertialMeasurement();
    measurement_heap_instantiated++;
    minimum_prealloc_level = 0;
  }
  else {
    return_value = preallocd_measurements.dequeue();
    minimum_prealloc_level = min((uint32_t) preallocd_measurements.size(), minimum_prealloc_level);
  }
  return return_value;
}



/**
* Reclaims the given InertialMeasurement so its memory can be re-used.
*
* At present, our criteria for preallocation is if the pointer address passed in
*   falls within the range of our __prealloc array. I see nothing "non-portable"
*   about this, it doesn't require a flag or class member, and it is fast to check.
* However, this strategy only works for types that are never used in DMA or code
*   execution on the STM32F4. It may work for other architectures (PIC32, x86?).
*   I also feel like it ought to be somewhat slower than a flag or member, but not
*   by such an amount that the memory savings are not worth the CPU trade-off.
* Consider writing all new cyclical queues with preallocated members to use this
*   strategy. Also, consider converting the most time-critical types to this strategy
*   up until we hit the boundaries of the STM32 CCM.
*                                 ---J. Ian Lindsay   Mon Apr 13 10:51:54 MST 2015
*
* @param InertialMeasurement* obj is the pointer to the object to be reclaimed.
*/
void LegendManager::reclaimMeasurement(InertialMeasurement* obj) {
  unsigned int obj_addr = ((uint32_t) obj);
  unsigned int pre_min  = ((uint32_t) INSTANCE->__prealloc);
  unsigned int pre_max  = pre_min + (sizeof(InertialMeasurement) * PREALLOCATED_IIU_MEASUREMENTS);

  if ((obj_addr < pre_max) && (obj_addr >= pre_min)) {
    // If we are in this block, it means obj was preallocated. wipe and reclaim it.
    obj->wipe();
    preallocd_measurements.insert(obj);
  }
  else {
    // We were created because our prealloc was starved. we are therefore a transient heap object.
    measurement_heap_freed++;
    delete obj;
  }
}





/****************************************************************************************************
*   ___ _              ___      _ _              _      _
*  / __| |__ _ ______ | _ ) ___(_) |___ _ _ _ __| |__ _| |_ ___
* | (__| / _` (_-<_-< | _ \/ _ \ | / -_) '_| '_ \ / _` |  _/ -_)
*  \___|_\__,_/__/__/ |___/\___/_|_\___|_| | .__/_\__,_|\__\___|
*                                          |_|
* Constructors/destructors, class initialization functions and so-forth...
****************************************************************************************************/
LegendManager::LegendManager() {
  __class_initializer();
  INSTANCE = this;

  reflection_mag.x = 1;
  reflection_mag.y = 1;
  reflection_mag.z = -1;

  reflection_acc.x = -1;
  reflection_acc.y = 1;
  reflection_acc.z = -1;

  reflection_gyr.x = -1;
  reflection_gyr.y = 1;
  reflection_gyr.z = -1;

    /* Populate all the static preallocation slots for measurements. */
  for (uint16_t i = 0; i < PREALLOCATED_IIU_MEASUREMENTS; i++) {
    __prealloc[i].wipe();
    preallocd_measurements.insert(&__prealloc[i]);
  }

  // Zero the ManuLegend.
  for (int i = 0; i < LEGEND_MGR_MAX_DATASET_SIZE; i++) {
    *(__dataset + i) = 0;
  }

  // Global frame data for MAP_STATE.
  _ptr_sequence = (uint32_t*) (__dataset + LEGEND_DATASET_OFFSET_SEQUENCE/4);
  _ptr_delta_t  = (float*)    (__dataset + LEGEND_DATASET_OFFSET_DELTA_T/4);

  *(_ptr_sequence) = 0;

  _legend_is_stable(true);  // Ick....
  operating_legend = new ManuLegend();
  operating_legend->sensorEnabled(true);
  //operating_legend->accNullGravity(true);
  operating_legend->accRaw(true);
  operating_legend->gyro(true);
  operating_legend->mag(true);
  operating_legend->orientation(true);
  operating_legend->temperature(true);
  if (operating_legend->finallize()) {
    local_log.concat("ManuLegend failed to finallize().\n");
  }
  reconfigure_data_map();
  setLegend(operating_legend);

  // Lastly, subscribe to Events.
  Kernel::getInstance()->subscribe(this);
}



/* This should probably never bee called. */
LegendManager::~LegendManager() {
  while (active_legends.hasNext()) active_legends.remove();
}


/*
* Return a ref to the indicated IIU, instantiating and validating it, if necessary.
* Assuming the caller passes an unsigned int in the range [0, 17), this fxn has
*   absolutely no excuse for returning NULL, since we built these objects when CPLDDriver
*   was constructed.
*/
IIU* LegendManager::fetchIIU(uint8_t idx) {
  if (idx > 16) {
    local_log.concatf("LegendManager::fetchIIU(%d):  We should crash, but will return mod-17 instead.\n", idx);
    Kernel::log(&local_log);
  }
  return &iius[idx % LEGEND_DATASET_IIU_COUNT];
}


int8_t LegendManager::init_iiu(uint8_t idx) {
  iius[idx].init();
  return 0;
}


/* Read the given IMU. */
int8_t LegendManager::refreshIMU(uint8_t idx) {
  if (idx > 16) return -1;

  iius[idx].readSensor();
  return 0;
}


/* Read all IMUs. */
int8_t LegendManager::refreshIMU() {

  if (last_imu_read) {
    // We are alrady doing something to the IMUs, Need to wait.
    if (getVerbosity() > 2) {
      local_log.concat("LegendManager tried to do two large IMU operations at once. Doing nothing. Be patient.\n");
      Kernel::log(&local_log);
    }
    return -1;
  }

  if (event_iiu_read.enableSchedule(true)) {
    // We are alrady doing something to the IMUs, Need to wait.
    local_log.concat("Tried to refresh all IMUs while the schedule to do the same thing is enabled. Doing nothing..\n");
    Kernel::log(&local_log);
    return -1;
  }

  event_iiu_read.fireNow();
  return 0;
}


/**
* Calling this method will cause the class to send the event that represents the
*   current state of the IMU data. This is the means by which we send IMU data to
*   a counterparty.
*
* @return non-zero on error.
*/
int8_t LegendManager::send_map_event() {
  event_legend_frame_ready.specific_target = NULL;
  if (operating_legend && _legend_sent()) {
    operating_legend->copy_frame();
    Kernel::staticRaiseEvent(&event_legend_frame_ready);
    return 0;
  }
  return -1;
}



/**
* Calling this fxn will cause the initial address assignment and dataset to be fed to the IIUs.
*
* @return non-zero on error.
*/
int8_t LegendManager::reconfigure_data_map() {
  uint16_t accumulated_offset = LEGEND_DATASET_GLOBAL_SIZE;
  for (uint8_t i = 0; i < LEGEND_DATASET_IIU_COUNT; i++) {
    // Configure the IIU...
    iius[i].setPositionAndAddress(i, CPLDDriver::imu_map[i].imu_addr, CPLDDriver::imu_map[i].mag_addr);

    /* Assign the ManuLegend specification to the IIU class, thereby giving the IIU class its pointers. */

    iius[i].assign_legend_pointers(
      (void*) (__dataset + accumulated_offset + LEGEND_DATASET_OFFSET_ACC      ),
      (void*) (__dataset + accumulated_offset + LEGEND_DATASET_OFFSET_GYR      ),
      (void*) (__dataset + accumulated_offset + LEGEND_DATASET_OFFSET_MAG      ),
      (void*) (__dataset + accumulated_offset + LEGEND_DATASET_OFFSET_VEL      ),
      (void*) (__dataset + accumulated_offset + LEGEND_DATASET_OFFSET_NULL_GRAV),
      (void*) (__dataset + accumulated_offset + LEGEND_DATASET_OFFSET_POSITION ),
      (void*) (__dataset + accumulated_offset + LEGEND_DATASET_OFFSET_QUAT     ),
      (void*) (__dataset + accumulated_offset + LEGEND_DATASET_OFFSET_TEMP     ),
      (void*) (__dataset + accumulated_offset + LEGEND_DATASET_OFFSET_SC_ACC   ),
      (void*) (__dataset + accumulated_offset + LEGEND_DATASET_OFFSET_SC_GYR   ),
      (void*) (__dataset + accumulated_offset + LEGEND_DATASET_OFFSET_SC_MAG   ),
      (void*) (__dataset + accumulated_offset + LEGEND_DATASET_OFFSET_SC_TMEP  )
    );
    accumulated_offset += LEGEND_DATASET_PER_IMU_SIZE;

    iius[i].nullGyroError(true);   // No reason to not do this...
    iius[i].nullifyGravity(false);
  }
  return 0;
}



// Calling causes a pointer dance that reconfigures the data we send to the host.
// Don't do anything unless the legend is stable. This is concurrency-control.
int8_t LegendManager::setLegend(ManuLegend* nu_legend) {
  if (_legend_is_stable()) {
    // Only reconfigure if stable.
    _legend_is_stable(false);   // Mark as unstable.
    _legend_sent(false);

    if (!nu_legend->finallized()) {
      // Finallize the ManuLegend prior to installing it.
      nu_legend->finallize();
    }

    bool should_enable_pid = false;
    if (event_legend_frame_ready.isScheduled()) {
      event_legend_frame_ready.enableSchedule(false);
      should_enable_pid = true;
    }


    // Store a pointer to our dataset in the event that is to carry them.
    // It is important that this argument NOT be reaped.
    event_legend_frame_ready.abort();      // We don't want this to proc while the dataset is in flux.
    event_legend_frame_ready.clearArgs();
    event_legend_frame_ready.addArg((void*) nu_legend->dataset_local, nu_legend->datasetSize());

    // Now we need to declare the new IMU Legend to the rest of the system. We will not re-enable
    //   the frame broadcast until the callback for this event happens. This assures that the message
    //   order to anyone listening is what we intend.
    StringBuilder* legend_string = new StringBuilder();
    nu_legend->formLegendString(legend_string);
    ManuvrRunnable* legend_broadcast     = Kernel::returnEvent(DIGITABULUM_MSG_IMU_LEGEND);
    legend_broadcast->originator      = (EventReceiver*) this;
    legend_broadcast->specific_target = nu_legend->owner;
    legend_broadcast->priority        = EVENT_PRIORITY_LOWEST + 1;
    legend_broadcast->markArgForReap(legend_broadcast->addArg(legend_string), true);

    if (should_enable_pid) {
      event_legend_frame_ready.enableSchedule(true);
    }
    return 0;
  }
  return -1;
}




/**
* Debug support function.
*
* @return a pointer to a string constant.
*/
const char* LegendManager::getReceiverName() {  return "LegendManager";  }


void LegendManager::printDebug(StringBuilder *output) {
  if (output == NULL) return;
  EventReceiver::printDebug(output);
  if (getVerbosity() > 0) {
    // Print just the aggregate sample count and return.
  }


  if (getVerbosity() > 3) {
    output->concatf("--- __dataset location  0x%08x\n", (uint32_t) __dataset);
    output->concatf("--- __prealloc location 0x%08x\n", (uint32_t) __prealloc);
    output->concatf("--- __IIU location      0x%08x\n", (uint32_t) iius);
    output->concatf("--- INSTANCE location   0x%08x\n---\n", (uint32_t) INSTANCE);
  }

  float grav_consensus = 0.0;
  output->concat("--- Intertial integration units:\n");
  for (uint8_t i = 0; i < 17; i++) {
    grav_consensus += iius[i].grav_scalar;
  }
  grav_consensus /= 17;
  output->concatf("--- Gravity consensus:  %.4fg\n",  (double) grav_consensus);

  output->concatf("--- Sequence number     %u\n",    (unsigned long) *(_ptr_sequence));
  output->concatf("--- Max quat proc       %u\n",    IIU::max_quats_per_event);
  output->concatf("--- Sequence number     %u\n",    (unsigned long) *(_ptr_sequence));
  output->concatf("--- Delta-t             %2.5f\n---\n", (double) *(_ptr_delta_t));

  if (getVerbosity() > 3) {
    output->concatf("--- MAX_DATASET_SIZE    %u\n",    (unsigned long) LEGEND_MGR_MAX_DATASET_SIZE);
  }
  if (getVerbosity() > 5) {
    event_legend_frame_ready.printDebug(output);
    event_iiu_read.printDebug(output);
  }

  output->concatf("--- prealloc starves    %u\n--- minimum_prealloc    %u\n", (unsigned long) prealloc_starves, (unsigned long) minimum_prealloc_level);
  output->concatf("--- Measurement queue info\n---\t Instantiated %u \t Freed: %u \t Prealloc queue depth: %d\n---\n", measurement_heap_instantiated, measurement_heap_freed, preallocd_measurements.size());

  output->concat("--- Intertial integration units:\n");
  for (uint8_t i = 0; i < 17; i++) {
    output->concatf("\tIIU %d\t ", i);
    iius[i].printBrief(output);
  }
  output->concat("\n");
}



uint32_t LegendManager::totalSamples() {
  uint32_t return_value = 0;
  for (uint8_t i = 0; i < LEGEND_DATASET_IIU_COUNT; i++) {
    return_value += iius[i].totalSamples();
  }
  return return_value;
}



/****************************************************************************************************
*  ▄▄▄▄▄▄▄▄▄▄▄  ▄               ▄  ▄▄▄▄▄▄▄▄▄▄▄  ▄▄        ▄  ▄▄▄▄▄▄▄▄▄▄▄  ▄▄▄▄▄▄▄▄▄▄▄
* ▐░░░░░░░░░░░▌▐░▌             ▐░▌▐░░░░░░░░░░░▌▐░░▌      ▐░▌▐░░░░░░░░░░░▌▐░░░░░░░░░░░▌
* ▐░█▀▀▀▀▀▀▀▀▀  ▐░▌           ▐░▌ ▐░█▀▀▀▀▀▀▀▀▀ ▐░▌░▌     ▐░▌ ▀▀▀▀█░█▀▀▀▀ ▐░█▀▀▀▀▀▀▀▀▀
* ▐░▌            ▐░▌         ▐░▌  ▐░▌          ▐░▌▐░▌    ▐░▌     ▐░▌     ▐░▌
* ▐░█▄▄▄▄▄▄▄▄▄    ▐░▌       ▐░▌   ▐░█▄▄▄▄▄▄▄▄▄ ▐░▌ ▐░▌   ▐░▌     ▐░▌     ▐░█▄▄▄▄▄▄▄▄▄
* ▐░░░░░░░░░░░▌    ▐░▌     ▐░▌    ▐░░░░░░░░░░░▌▐░▌  ▐░▌  ▐░▌     ▐░▌     ▐░░░░░░░░░░░▌
* ▐░█▀▀▀▀▀▀▀▀▀      ▐░▌   ▐░▌     ▐░█▀▀▀▀▀▀▀▀▀ ▐░▌   ▐░▌ ▐░▌     ▐░▌      ▀▀▀▀▀▀▀▀▀█░▌
* ▐░▌                ▐░▌ ▐░▌      ▐░▌          ▐░▌    ▐░▌▐░▌     ▐░▌               ▐░▌
* ▐░█▄▄▄▄▄▄▄▄▄        ▐░▐░▌       ▐░█▄▄▄▄▄▄▄▄▄ ▐░▌     ▐░▐░▌     ▐░▌      ▄▄▄▄▄▄▄▄▄█░▌
* ▐░░░░░░░░░░░▌        ▐░▌        ▐░░░░░░░░░░░▌▐░▌      ▐░░▌     ▐░▌     ▐░░░░░░░░░░░▌
*  ▀▀▀▀▀▀▀▀▀▀▀          ▀          ▀▀▀▀▀▀▀▀▀▀▀  ▀        ▀▀       ▀       ▀▀▀▀▀▀▀▀▀▀▀
*
* These are overrides from EventReceiver interface...
****************************************************************************************************/

/**
* There is a NULL-check performed upstream for the scheduler member. So no need
*   to do it again here.
*
* @return 0 on no action, 1 on action, -1 on failure.
*/
int8_t LegendManager::bootComplete() {
  EventReceiver::bootComplete();

  /* Get ready for a silly pointer dance....
  *  This is an argument-heavy event, and we will be using it ALOT. So we build the Event arguments
  *    once, and then change the data at the location being pointed at, and not the pointers in the
  *    arguments. Technically, we could make this even faster by addingg a new type for Vector3<float>**,
  *    but this will be a serious undertaking. We should ultimately implement ** types with a flag, not
  *    a type_code.
  *    Bassnectar - 04. You &amp; Me ft. W. Darling.mp3
  *    Knife Party - 04. EDM Trend Machine.mp3
  *    Bassnectar - 04 - Boomerang.mp3
  *    Bassnectar - 01. F.U.N..mp3
  *    ---J. Ian Lindsay   Thu Apr 09 04:04:41 MST 2015
  */
  event_iiu_read.repurpose(DIGITABULUM_MSG_IMU_READ);
  event_iiu_read.isManaged(true);
  event_iiu_read.specific_target = (EventReceiver*) this;
  event_iiu_read.originator      = (EventReceiver*) this;
  event_iiu_read.priority        = EVENT_PRIORITY_LOWEST;
  event_iiu_read.alterSchedulePeriod(20);
  event_iiu_read.alterScheduleRecurrence(-1);
  event_iiu_read.autoClear(false);
  event_iiu_read.enableSchedule(false);

  // Build some pre-formed Events.
  event_legend_frame_ready.repurpose(DIGITABULUM_MSG_IMU_MAP_STATE);
  event_legend_frame_ready.isManaged(true);
  event_legend_frame_ready.specific_target = NULL; //(EventReceiver*) this;
  event_legend_frame_ready.originator      = (EventReceiver*) this;
  event_legend_frame_ready.priority        = EVENT_PRIORITY_LOWEST;
  event_legend_frame_ready.alterSchedulePeriod(25);
  event_legend_frame_ready.alterScheduleRecurrence(-1);
  event_legend_frame_ready.autoClear(false);
  event_legend_frame_ready.enableSchedule(false);

  return 1;
}





/**
* If we find ourselves in this fxn, it means an event that this class built (the argument)
*   has been serviced and we are now getting the chance to see the results. The argument
*   to this fxn will never be NULL.
*
* Depending on class implementations, we might choose to handle the completed Event differently. We
*   might add values to event's Argument chain and return RECYCLE. We may also free() the event
*   ourselves and return DROP. By default, we will return REAP to instruct the Kernel
*   to either free() the event or return it to it's preallocate queue, as appropriate. If the event
*   was crafted to not be in the heap in its own allocation, we will return DROP instead.
*
* @param  event  The event for which service has been completed.
* @return A callback return code.
*/
int8_t LegendManager::callback_proc(ManuvrRunnable *event) {
  /* Setup the default return code. If the event was marked as mem_managed, we return a DROP code.
     Otherwise, we will return a REAP code. Downstream of this assignment, we might choose differently. */
  int8_t return_value = event->kernelShouldReap() ? EVENT_CALLBACK_RETURN_REAP : EVENT_CALLBACK_RETURN_DROP;

  /* Some class-specific set of conditionals below this line. */
  switch (event->event_code) {
    case DIGITABULUM_MSG_IMU_READ:
      switch (last_imu_read) {
        case 0:
        case 1:
        case 2:
        case 3:
        case 4:
        case 5:
        case 6:
        case 7:
        case 8:
        case 9:
        case 10:
        case 11:
        case 12:
        case 13:
        case 14:
        case 15:
          last_imu_read++;
          return EVENT_CALLBACK_RETURN_RECYCLE;

        case 16:
          last_imu_read = 0;
          break;

        default:
          if (getVerbosity() > 2) local_log.concat("LegendManager::callback_proc(IMU_READ): Bad arg\n");
          last_imu_read = 0;
          break;
      }
      break;


    case DIGITABULUM_MSG_IMU_INIT:
      switch (last_imu_read) {
        case 0:
        case 1:
        case 2:
        case 3:
        case 4:
        case 5:
        case 6:
        case 7:
        case 8:
        case 9:
        case 10:
        case 11:
        case 12:
        case 13:
        case 14:
        case 15:
          last_imu_read++;
          if (getVerbosity() > 6) local_log.concat("LegendManager::callback_proc(IMU_INIT): RECYCLING\n");
          // We still have IMUs left to deal with. Recycle the event...
          return EVENT_CALLBACK_RETURN_RECYCLE;
        case 16:
          last_imu_read = 0;
          if (getVerbosity() > 6) local_log.concat("LegendManager::callback_proc(IMU_INIT): DROPPING\n");
          break;

        default:
          if (getVerbosity() > 2) local_log.concat("LegendManager::callback_proc(IMU_READ): Bad arg\n");
          last_imu_read = 0;
          break;
      }
      break;

    case DIGITABULUM_MSG_IMU_LEGEND:
      // We take this as an indication that our notice of altered Legend was sent.
      _legend_sent(true);
      break;

    case DIGITABULUM_MSG_IMU_MAP_STATE:
      *(_ptr_sequence) = *(_ptr_sequence) + 1;
      if (operating_legend && _legend_sent()) {
        operating_legend->copy_frame();
        Kernel::staticRaiseEvent(&event_legend_frame_ready);
        return 0;
      }
      break;

    case DIGITABULUM_MSG_IMU_QUAT_CRUNCH:
      {
        uint8_t temp_uint_8;
        if (0 == event->getArgAs(&temp_uint_8)) {
          if (iius[temp_uint_8 % 17].has_quats_left()) {
            return_value = EVENT_CALLBACK_RETURN_RECYCLE;
          }
        }
        else {
          local_log.concat("LegendManager::callback_proc(): QUAT crunch had no argument?!.\n");
        }
      }
      break;

    default:
      if (getVerbosity() > 5) {
        local_log.concat("LegendManager::callback_proc(): Default case.\n");
        event->printDebug(&local_log);
        Kernel::log(&local_log);
      }
      break;
  }

  if (local_log.length() > 0) {    Kernel::log(&local_log);  }
  return return_value;
}




int8_t LegendManager::notify(ManuvrRunnable *active_event) {
  int8_t return_value = 0;
  uint8_t temp_uint_8 = 0;

  /* Some class-specific set of conditionals below this line. */
  switch (active_event->event_code) {
    case DIGITABULUM_MSG_IMU_READ:
      iius[last_imu_read].readSensor();
      return_value++;
      break;


    case MANUVR_MSG_SESS_ESTABLISHED:
      event_legend_frame_ready.delaySchedule(1100);     // Enable the periodic frame broadcast.
      {
        ManuvrRunnable *event = Kernel::returnEvent(DIGITABULUM_MSG_IMU_INIT);
        event->addArg((uint8_t) 4);  // Set the desired init stage.
        event->priority = 0;
        raiseEvent(event);
      }
      event_iiu_read.delaySchedule(1000);  // Enable the periodic read after letting the dust settle.
      return_value++;
      break;

    case MANUVR_MSG_SESS_HANGUP:
      event_legend_frame_ready.enableSchedule(false);
      for (uint8_t i = 0; i < LEGEND_DATASET_IIU_COUNT; i++) {
        ManuvrRunnable *event = Kernel::returnEvent(DIGITABULUM_MSG_IMU_INIT);
        event->addArg((uint8_t) 4);  // Set the desired init stage.
        event->priority = 0;
        raiseEvent(event);
      }
      event_iiu_read.enableSchedule(false);    // Disable the periodic read.
      return_value++;
      break;

    case DIGITABULUM_MSG_IMU_INIT:
      /* This is a request (probably from elsewhere in this class) to move one-or-more
           IMUs into the given INIT stage. The argument forms are...
           None        A request to move all IMUs into the minimum meaningful INIT stage (INIT-1).
           uint8       A request to move all IMUs into the given INIT stage.
       */
      if (0 == active_event->argCount()) {
        if (last_imu_read > 16) {
          if (getVerbosity() > 1) local_log.concat("MSG_IMU_INIT: last_imu_read > 16.\n");
        }
        else {
          iius[last_imu_read].init();
          return_value++;
        }
      }
      else if (0 == active_event->getArgAs(&temp_uint_8)) {
        // If the arg was present, we interpret this as a specified INIT stage...
        if (temp_uint_8 > 16) {
          if (getVerbosity() > 1) local_log.concat("MSG_IMU_INIT had an IMU idx > 16.\n");
        }
        else {
          iius[last_imu_read].state_pass_through(temp_uint_8);
          return_value++;
        }
      }
      break;

    case DIGITABULUM_MSG_IMU_MAP_STATE:
      //if (0 == active_event->argCount()) {
        // No args means a request. Send it.
      //  send_map_event();
      //  return_value++;
      //}
      break;


    case DIGITABULUM_MSG_CPLD_RESET_COMPLETE:
      {
        if (getVerbosity() > 3) local_log.concatf("Initializing IMUs...\n");

        // Range-bind everything....
        for (uint8_t i = 0; i < 17; i++) {
          iius[i].rangeBind(true);
        }

        // Fire the event to put the IMUs into INIT-1.
        ManuvrRunnable *event = Kernel::returnEvent(DIGITABULUM_MSG_IMU_INIT);
        raiseEvent(event);
      }
      return_value++;
      break;

    case DIGITABULUM_MSG_IMU_TAP:
      if (0 == active_event->argCount()) {
        // Somthing wants the thresholds for all configured taps.
      }
      else {
        // Otherwise, it means we've emitted the event. No need to respond.
      }
      break;

    case DIGITABULUM_MSG_IMU_QUAT_CRUNCH:
      if (0 == active_event->getArgAs(&temp_uint_8)) {
        if (temp_uint_8 > 16) {
          if (getVerbosity() > 1) local_log.concat("QUAT_CRUNCH had an IMU idx > 16.\n");
        }
        else {
          iius[temp_uint_8].MadgwickQuaternionUpdate();
        }
        return_value++;
      }
      else {
        if (getVerbosity() > 2) local_log.concatf("QUAT_CRUNCH handler (IIU %u) got a bad return from an Arg..\n", temp_uint_8);
      }
      break;

    default:
      return_value += EventReceiver::notify(active_event);
      break;
  }

  if (local_log.length() > 0) {    Kernel::log(&local_log);  }
  return return_value;
}



/****************************************************************************************************
*  ▄▄▄▄▄▄▄▄▄▄   ▄▄▄▄▄▄▄▄▄▄▄  ▄▄▄▄▄▄▄▄▄▄   ▄         ▄  ▄▄▄▄▄▄▄▄▄▄▄
* ▐░░░░░░░░░░▌ ▐░░░░░░░░░░░▌▐░░░░░░░░░░▌ ▐░▌       ▐░▌▐░░░░░░░░░░░▌
* ▐░█▀▀▀▀▀▀▀█░▌▐░█▀▀▀▀▀▀▀▀▀ ▐░█▀▀▀▀▀▀▀█░▌▐░▌       ▐░▌▐░█▀▀▀▀▀▀▀▀▀
* ▐░▌       ▐░▌▐░▌          ▐░▌       ▐░▌▐░▌       ▐░▌▐░▌
* ▐░▌       ▐░▌▐░█▄▄▄▄▄▄▄▄▄ ▐░█▄▄▄▄▄▄▄█░▌▐░▌       ▐░▌▐░▌ ▄▄▄▄▄▄▄▄
* ▐░▌       ▐░▌▐░░░░░░░░░░░▌▐░░░░░░░░░░▌ ▐░▌       ▐░▌▐░▌▐░░░░░░░░▌
* ▐░▌       ▐░▌▐░█▀▀▀▀▀▀▀▀▀ ▐░█▀▀▀▀▀▀▀█░▌▐░▌       ▐░▌▐░▌ ▀▀▀▀▀▀█░▌
* ▐░▌       ▐░▌▐░▌          ▐░▌       ▐░▌▐░▌       ▐░▌▐░▌       ▐░▌
* ▐░█▄▄▄▄▄▄▄█░▌▐░█▄▄▄▄▄▄▄▄▄ ▐░█▄▄▄▄▄▄▄█░▌▐░█▄▄▄▄▄▄▄█░▌▐░█▄▄▄▄▄▄▄█░▌
* ▐░░░░░░░░░░▌ ▐░░░░░░░░░░░▌▐░░░░░░░░░░▌ ▐░░░░░░░░░░░▌▐░░░░░░░░░░░▌
*  ▀▀▀▀▀▀▀▀▀▀   ▀▀▀▀▀▀▀▀▀▀▀  ▀▀▀▀▀▀▀▀▀▀   ▀▀▀▀▀▀▀▀▀▀▀  ▀▀▀▀▀▀▀▀▀▀▀
*
* Code in here only exists for as long as it takes to debug something. Don't write against these.
****************************************************************************************************/

void LegendManager::procDirectDebugInstruction(StringBuilder *input) {
  char* str = input->position(0);

  uint8_t temp_byte = 0;
  if (*(str) != 0) {
    temp_byte = atoi((char*) str+1);
  }

  StringBuilder parse_mule;

  switch (*(str)) {
    case 'v':
      parse_mule.concat(str);
      local_log.concatf("parse_mule split (%s) into %d positions.\n", str, parse_mule.split(","));
      parse_mule.drop_position(0);
      if (temp_byte < 17) {
        if (parse_mule.count() > 0) {
          int temp_int = parse_mule.position_as_int(0);
          iius[temp_byte].setVerbosity(temp_int);
        }
        local_log.concatf("Verbosity on IIU %d is %d.\n", temp_byte, iius[temp_byte].getVerbosity());
      }
      break;

    case 'i':
      if (1 == temp_byte) {
        local_log.concatf("The IIU preallocated measurements are stored at 0x%08x.\n", (uint32_t) __prealloc);
      }
      else if (2 == temp_byte) {
        if (operating_legend) {
          operating_legend->printDebug(&local_log);
        }
      }
      else {
        int8_t old_verbosity = getVerbosity();
        if (temp_byte && (temp_byte < 7)) setVerbosity(temp_byte);
        printDebug(&local_log);
        if (temp_byte && (temp_byte < 7)) setVerbosity(old_verbosity);
      }
      break;

    case '-':
      if (operating_legend) {
        operating_legend->copy_frame();
        local_log.concat("Frame copied.\n");
        operating_legend->printDataset(&local_log);
      }
      break;
    case '+':
      if (temp_byte < 17) {
        iius[temp_byte].dumpPointers(&local_log);
      }
      break;


    // IMU DEBUG //////////////////////////////////////////////////////////////////
    case 'c':
      if (temp_byte < 17) {
        iius[temp_byte].printDebug(&local_log);
      }
      break;

    // IMU STATE CONTROL //////////////////////////////////////////////////////////
    case 'g':
      if (255 == temp_byte) {
        local_log.concat("Syncing all IIUs...\n");
        for (uint8_t i = 0; i < 17; i++) {
          iius[i].sync();
        }
      }
      else if (temp_byte < 17) {
        iius[temp_byte].sync();
        local_log.concatf("Syncing IIU %d.\n", temp_byte);
      }
      break;

    case 's':
      parse_mule.concat(str);
      parse_mule.split(",");
      parse_mule.drop_position(0);
      if (parse_mule.count() > 0) {
        int temp_int = parse_mule.position_as_int(0);

        if (255 == temp_byte) {
          for (uint8_t i = 0; i < 17; i++) {
            iius[i].setOperatingState(temp_int);
          }
        }
        else if (temp_byte < 17) {
          local_log.concatf("Setting the state of IMU %d to %d\n", temp_byte, temp_int);
          iius[temp_byte].setOperatingState(temp_int);
        }

      }
      break;

    case 'k':
      if ((temp_byte < 6) && (temp_byte >= 0)) {
        ManuvrRunnable *event = Kernel::returnEvent(DIGITABULUM_MSG_IMU_INIT);
        event->addArg((uint8_t) temp_byte);  // Set the desired init stage.
        event->priority = 0;
        raiseEvent(event);
        local_log.concatf("Broadcasting IMU_INIT for stage %u...\n", temp_byte);
      }
      else {
        local_log.concatf("Illegal INIT stage: %u\n", temp_byte);
      }
      break;

    case 'r':
      if (255 == temp_byte) {
        local_log.concat("Reseting all IIUs...\n");
        for (uint8_t i = 0; i < 17; i++) {
          iius[i].reset();
        }
      }
      else if (temp_byte < 17) {
        local_log.concatf("Resetting IIU %d.\n", temp_byte);
        iius[temp_byte].reset();
      }
      break;


    case 'T':
    case 't':
      if (temp_byte < 17) {
        ManuvrRunnable *event = Kernel::returnEvent((*(str) == 'T') ? DIGITABULUM_MSG_IMU_DOUBLE_TAP : DIGITABULUM_MSG_IMU_TAP);
        event->originator      = (EventReceiver*) this;
        event->addArg((uint8_t) temp_byte);
        Kernel::staticRaiseEvent(event);
        local_log.concatf("Sent %stap event for IMU %d.\n", ((*(str) == 'T') ? "double ":""), temp_byte);
      }
      break;

    case 'q':
      if (temp_byte < 17) {
        ManuvrRunnable *event = Kernel::returnEvent(DIGITABULUM_MSG_IMU_QUAT_CRUNCH);
        event->specific_target = (EventReceiver*) this;
        event->originator      = NULL;
        event->addArg((uint8_t) temp_byte);
        Kernel::staticRaiseEvent(event);
        local_log.concatf("Running quat on IIU %d.\n", temp_byte);
      }
      break;



    // IMU DATA ///////////////////////////////////////////////////////////////////
    case 'j':
      switch (*(str+1)) {
        case '0':
          reflection_acc.x = -1;
          reflection_acc.y = 1;
          reflection_acc.z = -1;
          reflection_gyr.set(-1, 1, -1);
          reflection_mag.x = 1;
          reflection_mag.y = 1;
          reflection_mag.z = -1;
          break;
        case 'm':
          if (*(str+2) == 'x')      reflection_mag.x *= -1;
          else if (*(str+2) == 'y') reflection_mag.y *= -1;
          else if (*(str+2) == 'z') reflection_mag.z *= -1;
          break;

        case 'a':
          if (*(str+2) == 'x')      reflection_acc.x *= -1;
          else if (*(str+2) == 'y') reflection_acc.y *= -1;
          else if (*(str+2) == 'z') reflection_acc.z *= -1;
          break;

        case 'g':
          if (*(str+2) == 'x')      reflection_gyr.x *= -1;
          else if (*(str+2) == 'y') reflection_gyr.y *= -1;
          else if (*(str+2) == 'z') reflection_gyr.z *= -1;
          break;
      }
      local_log.concatf("Reflection vectors\n\tMag (%d, %d, %d)\n\tAcc (%d, %d, %d)\n\tGyr (%d, %d, %d)\n", reflection_mag.x, reflection_mag.y, reflection_mag.z, reflection_acc.x, reflection_acc.y, reflection_acc.z, reflection_gyr.x, reflection_gyr.y, reflection_gyr.z);
      break;

    case '[':
    case ']':
      if (255 == temp_byte) {
        local_log.concatf("%sabling spherical abberation correction on all IIUs.\n", ((*(str) == ']') ? "En":"Dis"));
        for (uint8_t i = 0; i < 17; i++) {
          iius[i].correctSphericalAbberation((*(str) == ']'));
        }
      }
      else if (temp_byte < 17) {
        iius[temp_byte].correctSphericalAbberation((*(str) == ']'));
        local_log.concatf("%sabling spherical abberation correction on IIU %d.\n", ((*(str) == ']') ? "En":"Dis"), temp_byte);
      }
      break;

    case 'u':
    case 'U':
      if (255 == temp_byte) {
        local_log.concatf("%sabling (clean-mag-is-zero) on all IIUs.\n", ((*(str) == 'U') ? "En":"Dis"));
        for (uint8_t i = 0; i < 17; i++) {
          iius[i].cleanMagZero((*(str) == 'U'));
        }
      }
      else if (temp_byte < 17) {
        iius[temp_byte].cleanMagZero((*(str) == 'U'));
        local_log.concatf("%sabling (clean-mag-is-zero) on IIU %d.\n", ((*(str) == 'U') ? "En":"Dis"), temp_byte);
      }
      break;

    case 'w':
    case 'W':
      if (255 == temp_byte) {
        local_log.concatf("%sabling mag data scrutiny on all IIUs.\n", ((*(str) == 'Z') ? "En":"Dis"));
        for (uint8_t i = 0; i < 17; i++) {
          iius[i].dropObviousBadMag((*(str) == 'Z'));
        }
      }
      else if (temp_byte < 17) {
        iius[temp_byte].dropObviousBadMag((*(str) == 'Z'));
        local_log.concatf("%sabling mag data scrutiny on IIU %d.\n", ((*(str) == 'Z') ? "En":"Dis"), temp_byte);
      }
      break;

    case 'z':
    case 'Z':
      if (255 == temp_byte) {
        local_log.concatf("%sabling autoscale on all IIUs.\n", ((*(str) == 'Z') ? "En":"Dis"));
        for (uint8_t i = 0; i < 17; i++) {
          iius[i].enableAutoscale((*(str) == 'Z'));
        }
      }
      else if (temp_byte < 17) {
        iius[temp_byte].enableAutoscale((*(str) == 'Z'));
        local_log.concatf("%sabling autoscale on IIU %d.\n", ((*(str) == 'Z') ? "En":"Dis"), temp_byte);
      }
      break;

    case 'n':
    case 'N':
      if (255 == temp_byte) {
        local_log.concatf("%sabling range-binding on all IIUs.\n", ((*(str) == 'N') ? "En":"Dis"));
        for (uint8_t i = 0; i < 17; i++) {
          iius[i].rangeBind((*(str) == 'N'));
        }
      }
      else if (temp_byte < 17) {
        iius[temp_byte].rangeBind((*(str) == 'N'));
        local_log.concatf("%sabling range-binding on IIU %d.\n", ((*(str) == 'N') ? "En":"Dis"), temp_byte);
      }
      break;

    case 'h':
    case 'H':
      if (255 == temp_byte) {
        local_log.concatf("%sabling quats on all IIUs.\n", ((*(str) == 'H') ? "En":"Dis"));
        for (uint8_t i = 0; i < 17; i++) {
          iius[i].processQuats((*(str) == 'H'));
        }
      }
      else if (temp_byte < 17) {
        iius[temp_byte].processQuats((*(str) == 'H'));
        local_log.concatf("%sabling quats on IIU %d.\n", ((*(str) == 'H') ? "En":"Dis"), temp_byte);
      }
      break;

    case 'x':
    case 'X':
      if (255 == temp_byte) {
        local_log.concatf("%sabling gyro error compensation on all IIUs.\n", ((*(str) == 'X') ? "En":"Dis"));
        for (uint8_t i = 0; i < 17; i++) {
          iius[i].nullGyroError((*(str) == 'X'));
        }
      }
      else if (temp_byte < 17) {
        iius[temp_byte].nullGyroError((*(str) == 'X'));
        local_log.concatf("%sabling gyro error compensation on IIU %d.\n", ((*(str) == 'X') ? "En":"Dis"), temp_byte);
      }
      break;

    case 'm':
    case 'M':
      if (255 == temp_byte) {
        local_log.concatf("%sabling gravity nullification on all IIUs.\n", ((*(str) == 'M') ? "En":"Dis"));
        for (uint8_t i = 0; i < 17; i++) {
          iius[i].nullifyGravity((*(str) == 'M'));
        }
      }
      else if (temp_byte < 17) {
        iius[temp_byte].nullifyGravity((*(str) == 'M'));
        local_log.concatf("%sabling gravity nullification on IIU %d.\n", ((*(str) == 'M') ? "En":"Dis"), temp_byte);
      }
      break;

    case 'y':
    case 'Y':
      if (255 == temp_byte) {
        local_log.concatf("%sabling bearing nullification on all IIUs.\n", ((*(str) == 'Y') ? "En":"Dis"));
        for (uint8_t i = 0; i < 17; i++) {
          iius[i].nullifyBearing((*(str) == 'Y'));
        }
      }
      else if (temp_byte < 17) {
        iius[temp_byte].nullifyBearing((*(str) == 'Y'));
        local_log.concatf("%sabling bearing nullification on IIU %d.\n", ((*(str) == 'Y') ? "En":"Dis"), temp_byte);
      }
      break;

    case 'Q':
      local_log.concatf("Madgwick iterations to %d on all IIUs.\n", temp_byte);
      for (uint8_t i = 0; i < 17; i++) {
        iius[i].madgwickIterations(temp_byte);
      }
      break;



    case ',':
      IIU::max_quats_per_event = temp_byte;
      local_log.concatf("IIU class now runs a maximum of %u quats per event.\n", IIU::max_quats_per_event);
      break;

    case 'b':
      for (uint8_t i = 0; i < 17; i++) {
        iius[i].beta = (float)temp_byte * 0.1;
      }
      local_log.concatf("Beta value is now %f.\n", (double) (temp_byte * 0.1f));
      break;

    case 'L':
      for (uint8_t i = 0; i < 17; i++) {
        iius[i].setSampleRateProfile(temp_byte);
      }
      local_log.concatf("Moving to sample rate profile %d.\n", temp_byte);
      break;

    case 'o':
      for (uint8_t i = 0; i < 17; i++) {
        iius[i].setGyroBaseFiler(temp_byte);
      }
      local_log.concatf("Setting GYR base filter to %d.\n", temp_byte);
      break;

    case 'O':
      for (uint8_t i = 0; i < 17; i++) {
        iius[i].setAccelBaseFiler(temp_byte);
      }
      local_log.concatf("Setting ACC base filter to %d.\n", temp_byte);
      break;

    case 'a':
      if (255 == temp_byte) {
        refreshIMU();
      }
      else if (17 > temp_byte) {
        refreshIMU(temp_byte);
      }
      break;

    case 'd':
      if (255 == temp_byte) {
        event_legend_frame_ready.fireNow();  // Fire a single frame transmission.
        local_log.concat("We are manually firing the IMU frame broadcasts schedule.\n");
      }
      else if (254 == temp_byte) {
        event_legend_frame_ready.enableSchedule(true);  // Enable the periodic read.
        local_log.concat("Enabled frame broadcasts.\n");
      }
      else if (253 == temp_byte) {
        event_legend_frame_ready.printDebug(&local_log);
      }
      else if (252 == temp_byte) {
        send_map_event();
        local_log.concat("We are manually firing the IMU frame broadcasts schedule.\n");
      }
      else if (temp_byte) {
        event_legend_frame_ready.alterSchedulePeriod(temp_byte*10);
        local_log.concatf("Set periodic frame broadcast to once every %dms.\n", temp_byte*10);
      }
      else {
        event_legend_frame_ready.enableSchedule(false);  // Disable the periodic read.
        local_log.concat("Disabled frame broadcasts.\n");
      }
      break;

    case 'f':
      if (255 == temp_byte) {
        event_iiu_read.fireNow();
        local_log.concat("We are manually firing the IMU read schedule.\n");
      }
      else if (254 == temp_byte) {
        event_iiu_read.enableSchedule(true);
        local_log.concat("Enabled periodic readback.\n");
      }
      else if (temp_byte) {
        event_iiu_read.alterSchedulePeriod(temp_byte*10);
        local_log.concatf("Set periodic read schedule to once every %dms.\n", temp_byte*10);
      }
      else {
        event_iiu_read.enableSchedule(false);  // Disable the periodic read.
        local_log.concat("Disabled periodic readback.\n");
      }
      break;

    case 'p':
      {
        parse_mule.concat(str);
        parse_mule.split(",");
        parse_mule.drop_position(0);
        uint8_t start = (temp_byte < 17) ? temp_byte   : 0;
        uint8_t stop  = (temp_byte < 17) ? temp_byte+1 : 17;
        int temp_int  = (parse_mule.count() > 0) ? parse_mule.position_as_int(0) : 255;
        for (uint8_t i = start; i < stop; i++) {
          if (255 != temp_int) {  // The user wants to make a change..
            iius[i].enableProfiling(temp_int ? true:false);
          }
          local_log.concatf("Profiling IIU %d: %sabled.\n", i, (iius[i].enableProfiling() ? "en":"dis"));
        }
      }
      break;

    case 'e':
      if (temp_byte < 17) {
        iius[temp_byte].dumpPreformedElements(&local_log);
      }
      break;


    default:
      EventReceiver::procDirectDebugInstruction(input);
      break;
  }

  if (local_log.length() > 0) {    Kernel::log(&local_log);  }
}
