/*
File:   ManuLegend.h
Author: J. Ian Lindsay
Date:   2014.03.10

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


The ManuLegend is the filtering mechanism that we use to isolate glove data into
  what the system that instantiates it needs. It is the logical interface
  between the core sensor package and and any more-distant software that needs
  to know about the results generated by the sensors.

The ManuLegend is going to end up being a proximate driver of power managemnt.
  Many different ManuLegends can exist at once for different client classes and
  systems. The boolean union of all active ManuLegends directly translates to
  the minimum power draw on the glove. (Sensor activity, radio use, etc).

In Digitabulum r0, this class held 17 instances of the IIU class, each of which
  was an autonomous agent on the bus. While this strategy gave good results
  given r0's CPLD design, it made allocation of contiguous runs of memory
  impractical. To reap the most benefits from the CPLD improvements in r1, we
  will allocate register space for the IIU classes here, and pass a struct into
  the IIU class that contains the pointers to memory that the IIU class should
  assign to the sensor driver.
*/

#ifndef __DIGITABULUM_MANU_LEGEND_H_
#define __DIGITABULUM_MANU_LEGEND_H_

#include <Kernel.h>
#include "Integrator.h"
#include "../CPLDDriver/CPLDDriver.h"


#define  DATA_LEGEND_FLAGS_SENSOR_ACTIVE      0x0001   // If set, this ManuLegend will be included in the sum.
#define  DATA_LEGEND_FLAGS_REPORT_SEQUENCE    0x0002   //
#define  DATA_LEGEND_FLAGS_REPORT_DELTA_T     0x0004   //
#define  DATA_LEGEND_FLAGS_REPORT_GLOBAL_POS  0x0008   //

/*
* Bitmask flags for IMU data that makes its way into the map. This is the ManuLegend spec.
*/
#define  DATA_LEGEND_FLAGS_IIU_ACC            0x0001   //
#define  DATA_LEGEND_FLAGS_IIU_GYRO           0x0002   //
#define  DATA_LEGEND_FLAGS_IIU_MAG            0x0004   //
#define  DATA_LEGEND_FLAGS_IIU_TEMP           0x0008   //
#define  DATA_LEGEND_FLAGS_IIU_ORIENTATION    0x0010   //
#define  DATA_LEGEND_FLAGS_IIU_NULL_GRAV      0x0020   //
#define  DATA_LEGEND_FLAGS_IIU_VELOCITY       0x0040   //
#define  DATA_LEGEND_FLAGS_IIU_POSITION       0x0080   //
#define  DATA_LEGEND_FLAGS_IIU_SC_ACC         0x0100   //
#define  DATA_LEGEND_FLAGS_IIU_SC_GYRO        0x0200   //
#define  DATA_LEGEND_FLAGS_IIU_SC_MAG         0x0400   //
#define  DATA_LEGEND_FLAGS_IIU_SC_TEMPERATURE 0x0800   //


/*
* To make interpretation and access of the IMU data as linear as possible, we define a giant
*   pool of memory on the stack and pass pointers to the IIU classes, which treat them as
*   locations to write their data. Because we allocate in a single chunk, a full map read
*   operation can occur in a single loop without having to check types and lengths and such
*   for each discrete piece of data in the map (recipients of the data ought to already have
*   an IMU legend).
*
* When the Legend changes, we re-assign the pointers in the IIU classes to cause updates to
*   be transparently written to the new locations. We then alter the MAP_STATE event to
*   respect the new size (if it changed). Finally, we broadcast an event to confirm a change
*   in the Legend ahead of any further MAP_STATE events.
*
* Calculate the maximum size of the dataset...
*      36          // 3 vectors of 3 floats (one for each raw data type).
*      16          // A vector of 4 floats for quaternion.
*      12          // A vector of 3 floats for nulled-gravity.
*      12          // A vector of 3 floats for velocity.
*      12          // A vector of 3 floats for position.
*      16          // 4 uint32 fields for sample count.
*    +  4          // A float for temperature.
*    ------------
*     108 bytes
*
*     108 bytes
*    x 17 IIUs
*    ------------
*    1836 bytes for IMU data
*
*       4          // A sequence number for broadcasts. uint32
*       4          // A delta-t for broadcasts. float
*      12          // A vector of 3 floats for position.
*    +  4          // Reserved, for wiggle room later.
*    ------------------------------------------------------------
*      24 bytes for data global to a ManuLegend.
*
* Our maximum dataset size is therefore...
*   = overhead + IMU
*   = 24 + 1836
*   = 1860
*
* The worst thing about this strategy is that we have a resting memory usage equivilent to
*   the maximum size of a legend that we support. But since this might be a few KB, I've
*   deemed it worth the CPU trade-off (which is enormous). The offset map follows.
*                                           ---J. Ian Lindsay   Mon May 04 01:34:29 MST 2015
*
*
* Offset map (converting from legend spec (bitmasks) to pointers).
* ------------------------------------------------------------------------------------------
* The global legend data leads the IMU data, always.
* IMUs
*
* acc gyr mag
*
*
*
*
*/

/* These are offsets in the main data table that are globals. These lead the IIU data. */
#define LEGEND_DATASET_OFFSET_SEQUENCE   0
#define LEGEND_DATASET_OFFSET_DELTA_T    4
#define LEGEND_DATASET_OFFSET_G_POSITION 8
#define LEGEND_DATASET_OFFSET_RESRVD     20

#define LEGEND_DATASET_RESRVD_SIZE       4
#define LEGEND_DATASET_GLOBAL_SIZE      24

/* These are offsets in the main data table that relate to a given IIU. */
#define LEGEND_DATASET_OFFSET_QUAT      0
#define LEGEND_DATASET_OFFSET_ACC       16
#define LEGEND_DATASET_OFFSET_GYR       28
#define LEGEND_DATASET_OFFSET_MAG       40
#define LEGEND_DATASET_OFFSET_TEMP      52
#define LEGEND_DATASET_OFFSET_VEL       56
#define LEGEND_DATASET_OFFSET_SC_ACC    68
#define LEGEND_DATASET_OFFSET_SC_GYR    72
#define LEGEND_DATASET_OFFSET_SC_MAG    76
#define LEGEND_DATASET_OFFSET_SC_TMEP   80
#define LEGEND_DATASET_OFFSET_NULL_GRAV 84
#define LEGEND_DATASET_OFFSET_POSITION  96

#define LEGEND_DATASET_PER_IMU_SIZE     108
#define LEGEND_DATASET_IIU_COUNT        17


/* Therefore, the full map size is.... */
#define LEGEND_MGR_MAX_DATASET_SIZE     LEGEND_DATASET_GLOBAL_SIZE + (LEGEND_DATASET_PER_IMU_SIZE * LEGEND_DATASET_IIU_COUNT)


class ManuLegend {
  public:
    uint8_t*       dataset_global = nullptr;  // The dataset common to all ManuLegends.
    uint8_t*       dataset_local  = nullptr;  // The dataset the EventReceiver asked for.
    EventReceiver* owner          = nullptr;  // The owner of this ManuLegend.

    ManuLegend();
    ~ManuLegend();

    /*
    * Call this fxn to finalize our choices about the ManuLegend and allocate the memory.
    */
    int8_t finallize();
    inline bool finallized()            {  return (NULL != dataset_local);    };

    int8_t copy_frame();
    uint16_t datasetSize();
    void printDebug(StringBuilder *);
    void printDataset(StringBuilder *);
    void formLegendString(StringBuilder *);

    /* This is global data. */
    inline bool sensorEnabled()         {  return (_legend_flags & DATA_LEGEND_FLAGS_SENSOR_ACTIVE);     };     // Should this sensor be represented in map reports?
    inline bool positionGlobal()        {  return (_legend_flags & DATA_LEGEND_FLAGS_REPORT_GLOBAL_POS); };     // Global data: Should we return a global hand position?
    inline bool sequence()              {  return (_legend_flags & DATA_LEGEND_FLAGS_REPORT_SEQUENCE);   };     // Global data: Should we return a sequence number?
    inline bool deltaT()                {  return (_legend_flags & DATA_LEGEND_FLAGS_REPORT_DELTA_T);    };     // Global data: Should we return a deltaT since last frame?

    inline void sensorEnabled(bool en)  {  _legend_flags = (en) ? (_legend_flags | DATA_LEGEND_FLAGS_SENSOR_ACTIVE)     : (_legend_flags & ~(DATA_LEGEND_FLAGS_SENSOR_ACTIVE));      };
    inline void positionGlobal(bool en) {  _legend_flags = (en) ? (_legend_flags | DATA_LEGEND_FLAGS_REPORT_GLOBAL_POS) : (_legend_flags & ~(DATA_LEGEND_FLAGS_REPORT_GLOBAL_POS));  };
    inline void sequence(bool en)       {  _legend_flags = (en) ? (_legend_flags | DATA_LEGEND_FLAGS_REPORT_SEQUENCE)   : (_legend_flags & ~(DATA_LEGEND_FLAGS_REPORT_SEQUENCE));    };
    inline void deltaT(bool en)         {  _legend_flags = (en) ? (_legend_flags | DATA_LEGEND_FLAGS_REPORT_DELTA_T)    : (_legend_flags & ~(DATA_LEGEND_FLAGS_REPORT_DELTA_T));     };

    /* This is per-sensor data. */
    inline bool accRaw(uint8_t idx)             {  return (per_iiu_data[idx % LEGEND_DATASET_IIU_COUNT] & DATA_LEGEND_FLAGS_IIU_ACC           ); };
    inline bool gyro(uint8_t idx)               {  return (per_iiu_data[idx % LEGEND_DATASET_IIU_COUNT] & DATA_LEGEND_FLAGS_IIU_GYRO          ); };
    inline bool mag(uint8_t idx)                {  return (per_iiu_data[idx % LEGEND_DATASET_IIU_COUNT] & DATA_LEGEND_FLAGS_IIU_MAG           ); };
    inline bool temperature(uint8_t idx)        {  return (per_iiu_data[idx % LEGEND_DATASET_IIU_COUNT] & DATA_LEGEND_FLAGS_IIU_TEMP          ); };
    inline bool orientation(uint8_t idx)        {  return (per_iiu_data[idx % LEGEND_DATASET_IIU_COUNT] & DATA_LEGEND_FLAGS_IIU_ORIENTATION   ); };
    inline bool accNullGravity(uint8_t idx)     {  return (per_iiu_data[idx % LEGEND_DATASET_IIU_COUNT] & DATA_LEGEND_FLAGS_IIU_NULL_GRAV     ); };
    inline bool velocity(uint8_t idx)           {  return (per_iiu_data[idx % LEGEND_DATASET_IIU_COUNT] & DATA_LEGEND_FLAGS_IIU_VELOCITY      ); };
    inline bool position(uint8_t idx)           {  return (per_iiu_data[idx % LEGEND_DATASET_IIU_COUNT] & DATA_LEGEND_FLAGS_IIU_POSITION      ); };
    inline bool samplesAcc(uint8_t idx)         {  return (per_iiu_data[idx % LEGEND_DATASET_IIU_COUNT] & DATA_LEGEND_FLAGS_IIU_SC_ACC        ); };
    inline bool samplesGyro(uint8_t idx)        {  return (per_iiu_data[idx % LEGEND_DATASET_IIU_COUNT] & DATA_LEGEND_FLAGS_IIU_SC_GYRO       ); };
    inline bool samplesMag(uint8_t idx)         {  return (per_iiu_data[idx % LEGEND_DATASET_IIU_COUNT] & DATA_LEGEND_FLAGS_IIU_SC_MAG        ); };
    inline bool samplesTemperature(uint8_t idx) {  return (per_iiu_data[idx % LEGEND_DATASET_IIU_COUNT] & DATA_LEGEND_FLAGS_IIU_SC_TEMPERATURE); };

    inline void accRaw(uint8_t idx, bool en)             {   _internal_setter(idx, en, DATA_LEGEND_FLAGS_IIU_ACC           );  };     // Primary data:  Return for the given IIU?
    inline void gyro(uint8_t idx, bool en)               {   _internal_setter(idx, en, DATA_LEGEND_FLAGS_IIU_GYRO          );  };     // Primary data:  Return for the given IIU?
    inline void mag(uint8_t idx, bool en)                {   _internal_setter(idx, en, DATA_LEGEND_FLAGS_IIU_MAG           );  };     // Primary data:  Return for the given IIU?
    inline void temperature(uint8_t idx, bool en)        {   _internal_setter(idx, en, DATA_LEGEND_FLAGS_IIU_TEMP          );  };     // Primary data:  Return for the given IIU?
    inline void orientation(uint8_t idx, bool en)        {   _internal_setter(idx, en, DATA_LEGEND_FLAGS_IIU_ORIENTATION   );  };     // Inferred data: Return for the given IIU?
    inline void accNullGravity(uint8_t idx, bool en)     {   _internal_setter(idx, en, DATA_LEGEND_FLAGS_IIU_NULL_GRAV     );  };     // Inferred data: Return for the given IIU?
    inline void velocity(uint8_t idx, bool en)           {   _internal_setter(idx, en, DATA_LEGEND_FLAGS_IIU_VELOCITY      );  };     // Inferred data: Return for the given IIU?
    inline void position(uint8_t idx, bool en)           {   _internal_setter(idx, en, DATA_LEGEND_FLAGS_IIU_POSITION      );  };     // Inferred data: Return for the given IIU?
    inline void samplesAcc(uint8_t idx, bool en)         {   _internal_setter(idx, en, DATA_LEGEND_FLAGS_IIU_SC_ACC        );  };     // Sample counts: Return for the given IIU?
    inline void samplesGyro(uint8_t idx, bool en)        {   _internal_setter(idx, en, DATA_LEGEND_FLAGS_IIU_SC_GYRO       );  };     // Sample counts: Return for the given IIU?
    inline void samplesMag(uint8_t idx, bool en)         {   _internal_setter(idx, en, DATA_LEGEND_FLAGS_IIU_SC_MAG        );  };     // Sample counts: Return for the given IIU?
    inline void samplesTemperature(uint8_t idx, bool en) {   _internal_setter(idx, en, DATA_LEGEND_FLAGS_IIU_SC_TEMPERATURE);  };     // Sample counts: Return for the given IIU?

    /* This is per-sensor data, but changes ALL IIU classes in a single call. */
    void accRaw(bool en)             {   for (uint8_t idx = 0; idx < LEGEND_DATASET_IIU_COUNT; idx++) _internal_setter(idx, en, DATA_LEGEND_FLAGS_IIU_ACC           );  };     // Primary data:  Return for the given IIU?
    void gyro(bool en)               {   for (uint8_t idx = 0; idx < LEGEND_DATASET_IIU_COUNT; idx++) _internal_setter(idx, en, DATA_LEGEND_FLAGS_IIU_GYRO          );  };     // Primary data:  Return for the given IIU?
    void mag(bool en)                {   for (uint8_t idx = 0; idx < LEGEND_DATASET_IIU_COUNT; idx++) _internal_setter(idx, en, DATA_LEGEND_FLAGS_IIU_MAG           );  };     // Primary data:  Return for the given IIU?
    void temperature(bool en)        {   for (uint8_t idx = 0; idx < LEGEND_DATASET_IIU_COUNT; idx++) _internal_setter(idx, en, DATA_LEGEND_FLAGS_IIU_TEMP          );  };     // Primary data:  Return for the given IIU?
    void orientation(bool en)        {   for (uint8_t idx = 0; idx < LEGEND_DATASET_IIU_COUNT; idx++) _internal_setter(idx, en, DATA_LEGEND_FLAGS_IIU_ORIENTATION   );  };     // Inferred data: Return for the given IIU?
    void accNullGravity(bool en)     {   for (uint8_t idx = 0; idx < LEGEND_DATASET_IIU_COUNT; idx++) _internal_setter(idx, en, DATA_LEGEND_FLAGS_IIU_NULL_GRAV     );  };     // Inferred data: Return for the given IIU?
    void velocity(bool en)           {   for (uint8_t idx = 0; idx < LEGEND_DATASET_IIU_COUNT; idx++) _internal_setter(idx, en, DATA_LEGEND_FLAGS_IIU_VELOCITY      );  };     // Inferred data: Return for the given IIU?
    void position(bool en)           {   for (uint8_t idx = 0; idx < LEGEND_DATASET_IIU_COUNT; idx++) _internal_setter(idx, en, DATA_LEGEND_FLAGS_IIU_POSITION      );  };     // Inferred data: Return for the given IIU?
    void samplesAcc(bool en)         {   for (uint8_t idx = 0; idx < LEGEND_DATASET_IIU_COUNT; idx++) _internal_setter(idx, en, DATA_LEGEND_FLAGS_IIU_SC_ACC        );  };     // Sample counts: Return for the given IIU?
    void samplesGyro(bool en)        {   for (uint8_t idx = 0; idx < LEGEND_DATASET_IIU_COUNT; idx++) _internal_setter(idx, en, DATA_LEGEND_FLAGS_IIU_SC_GYRO       );  };     // Sample counts: Return for the given IIU?
    void samplesMag(bool en)         {   for (uint8_t idx = 0; idx < LEGEND_DATASET_IIU_COUNT; idx++) _internal_setter(idx, en, DATA_LEGEND_FLAGS_IIU_SC_MAG        );  };     // Sample counts: Return for the given IIU?
    void samplesTemperature(bool en) {   for (uint8_t idx = 0; idx < LEGEND_DATASET_IIU_COUNT; idx++) _internal_setter(idx, en, DATA_LEGEND_FLAGS_IIU_SC_TEMPERATURE);  };     // Sample counts: Return for the given IIU?


  private:
    uint16_t _legend_flags = 0;
    uint16_t ds_size       = 0;
    uint16_t per_iiu_data[LEGEND_DATASET_IIU_COUNT];

    inline void _internal_setter(uint8_t idx, bool en, uint16_t x) {  per_iiu_data[idx % LEGEND_DATASET_IIU_COUNT] = (en) ? (per_iiu_data[idx % LEGEND_DATASET_IIU_COUNT] | x) : (per_iiu_data[idx % LEGEND_DATASET_IIU_COUNT]  & ~(x));   };
};

#endif  // __DIGITABULUM_MANU_LEGEND_H_
