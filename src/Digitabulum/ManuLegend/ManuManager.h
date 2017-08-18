/*
File:   ManuManager.h
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
  will allocate register space for the LSM9DS1 classes here, and pass a struct into
  each instance that contains the pointers to memory that represents the registers.
*/

#ifndef __DIGITABULUM_MANU_MGR_H_
#define __DIGITABULUM_MANU_MGR_H_

#include <Kernel.h>
#include <DataStructures/RingBuffer.h>
#include <DataStructures/ElementPool.h>
#include "../CPLDDriver/CPLDDriver.h"
#include "../LSM9DS1/LSM9DS1.h"
#include "ManuLegend.h"
#include "ManuLegendPipe.h"
#include "Integrator.h"


/*
* These state flags are hosted by the EventReceiver. This may change in the future.
* Might be too much convention surrounding their assignment across inherritence.
*/
#define LEGEND_MGR_FLAGS_CHIRALITY_KNOWN       0x01   // Has the chirality been determined?
#define LEGEND_MGR_FLAGS_CHIRALITY_LEFT        0x02   // If so, is it a left hand?
#define LEGEND_MGR_FLAGS_IO_ON_HIGH_FRAME_AG   0x10   //
#define LEGEND_MGR_FLAGS_IO_ON_HIGH_FRAME_M    0x20   //
#define LEGEND_MGR_FLAGS_EMPTY_FRAME_CYCLE     0x40   //

#define LEGEND_MGR_FLAGS_CHIRALITY_MASK        0x03   // Mask that maps to the enum class.


#ifndef PREALLOCD_IMU_FRAMES
  #define PREALLOCD_IMU_FRAMES    10   // We retain this many frames.
#endif
#ifndef CONFIG_INTEGRATOR_Q_DEPTH
  #define CONFIG_INTEGRATOR_Q_DEPTH  PREALLOCD_IMU_FRAMES
#endif


enum class Chirality {
  UNKNOWN = 0,   // Interpretable as a bitmask...
  RIGHT   = 1,   // Bit 0: Chirality known
  LEFT    = 3    // Bit 1: Left-handed
};

/*
* Chirality invarient identifiers for fingers. We follow anatomical convention
* And consider the thumb to be digit 1.
*/
enum class Anatomical {
  METACARPALS = 0,
  DIGIT_1     = 1,
  DIGIT_2     = 2,
  DIGIT_3     = 3,
  DIGIT_4     = 4,
  DIGIT_5     = 5,
  UNKNOWN     = 6
};


/*
* The ManuManager is a system service that deals with interpreting the raw data generated by
*   Digitabulum's sensor package and passes them off to any number of instantiated ManuLegends.
*
* Like the CPLDDriver, this class implements BusOpCallback. IMU data is read directly
*   into this class.
*/
class ManuManager : public EventReceiver, public BusOpCallback {
  public:
    ManuManager(BusAdapter<SPIBusOp>*);
    ~ManuManager();

    /* Overrides from EventReceiver */
    void printDebug(StringBuilder*);
    int8_t notify(ManuvrMsg*);
    int8_t callback_proc(ManuvrMsg*);
    int8_t writeFrameToBuffer(StringBuilder*);
    #if defined(MANUVR_CONSOLE_SUPPORT)
      void procDirectDebugInstruction(StringBuilder*);
    #endif  //MANUVR_CONSOLE_SUPPORT

    /* Overrides from the BusOpCallback interface */
    int8_t io_op_callahead(BusOp*);
    int8_t io_op_callback(BusOp*);
    int8_t queue_io_job(BusOp*);


    void printHelp(StringBuilder*);
    void dumpPreformedElements(StringBuilder*);

    int8_t setLegend(ManuLegend*);
    inline ManuLegend* getActiveLegend() {    return &_root_leg;    }

    int8_t read_ag_frame();
    int8_t read_mag_frame();

    /* Expose our idea about handedness to other modules. */
    inline Chirality getChirality() {  return (Chirality) _er_flag(LEGEND_MGR_FLAGS_CHIRALITY_MASK);  };
    inline bool chiralityKnown() {     return _er_flag(LEGEND_MGR_FLAGS_CHIRALITY_KNOWN);  };

    inline bool debugFrameCycle() {           return (_er_flag(LEGEND_MGR_FLAGS_EMPTY_FRAME_CYCLE));           };
    inline void debugFrameCycle(bool nu) {    return (_er_set_flag(LEGEND_MGR_FLAGS_EMPTY_FRAME_CYCLE, nu));   };

    inline uint32_t totalSamples() {   return sample_count;   };


    static const char* chiralityString(Chirality);



  protected:
    /* Overrides from EventReceiver */
    int8_t attached();


  private:
    ManuvrMsg event_iiu_read;
    ManuvrMsg _event_integrator;
    ElementPool<SensorFrame> _frame_pool;

    CPLDDriver* _bus         = nullptr;   // This is the gateway to the hardware.
    ManuLegend _root_leg;                 // Data demand slots. Two for now.
    Integrator integrator;
    ManuLegendPipe _def_pipe;   // TODO: Cut once working.

    /* This is the dataset that we export. */
    uint8_t __dataset[LEGEND_MGR_MAX_DATASET_SIZE];

    // Used to direct data that we don't want in the legend until data-selection-by-NULL is implemented
    //   in the IIU class.
    uint32_t*       _ptr_sequence = nullptr;
    float*          _ptr_delta_t  = nullptr;
    uint32_t  sample_count       = 0;   // How many samples have we read since init?
    uint32_t  _frame_time_last   = 0;   // Used to track inter-frame time differences.

    uint8_t  max_quats_per_event = 2;   // Cuts down on overhead if load is high.

    /* The pool of SensorFrames is maintained by ManuManager. */
    void reclaimMeasurement(SensorFrame*);

    int8_t send_map_event();

    int8_t init_iius();
    int8_t read_identities();
    int8_t read_fifo_depth();

    /*
    * Accessors for autoscaling.
    */
    void enableAutoscale(SampleType, bool enabled);
    inline void enableAutoscale(bool enabled) {
      enableAutoscale(SampleType::ALL, enabled);
    };

    LSM9DS1* fetchIMU(uint8_t idx);

    void printIMURollCall(StringBuilder*);
    void printTemperatures(StringBuilder*);
    void printFIFOLevels(StringBuilder*);

    int set_chirality(Chirality);
    DigitPort  get_port_given_digit(Anatomical);
    Anatomical get_digit_given_port(DigitPort);

    int8_t calibrate_from_data_mag();
    int8_t calibrate_from_data_ag();


    /* Inlines for deriving address and IRQ bit offsets from index. */
    // Address of the inertial half of the LSM9DS1.
    inline uint8_t _intertial_addr(int idx) {   return ((idx % 17) + CPLD_REG_IMU_DM_P_I);   };
    // Address of the magnetic half of the LSM9DS1.
    inline uint8_t _magnetic_addr(int idx) {    return ((idx % 17) + CPLD_REG_IMU_DM_P_M);   };


    // These are preformed bus operations that address multiple IMUs...
    static SPIBusOp _preformed_read_i;
    static SPIBusOp _preformed_read_m;
    static SPIBusOp _preformed_read_temp;
    static SPIBusOp _preformed_fifo_read;
};

#endif  // __DIGITABULUM_MANU_MGR_H_
