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
  will allocate register space for the IIU classes here, and pass a struct into
  the IIU class that contains the pointers to memory that the IIU class should
  assign to the sensor driver.
*/

#ifndef __DIGITABULUM_MANU_MGR_H_
#define __DIGITABULUM_MANU_MGR_H_

#include <Kernel.h>
#include "../LSM9DS1/IIU.h"
#include "../CPLDDriver/CPLDDriver.h"
#include "ManuLegend.h"


#define  LEGEND_MGR_IIU_STATE_ABSENT         -1
#define  LEGEND_MGR_IIU_STATE_NOT_UPDATED    0
#define  LEGEND_MGR_IIU_STATE_UPDATED        1

/*
* These state flags are hosted by the EventReceiver. This may change in the future.
* Might be too much convention surrounding their assignment across inherritence.
*/
#define LEGEND_MGR_FLAGS_CHIRALITY_KNOWN       0x01   // Has the chirality been determined?
#define LEGEND_MGR_FLAGS_CHIRALITY_LEFT        0x02   // If so, is it a left hand?
#define LEGEND_MGR_FLAGS_LEGEND_STABLE         0x04   // If set, the map is not changing.
#define LEGEND_MGR_FLAGS_LEGEND_SENT           0x08   // If set, the map is clear-to-send.
#define LEGEND_MGR_FLAGS_IO_ON_HIGH_FRAME_AG   0x10   //
#define LEGEND_MGR_FLAGS_IO_ON_HIGH_FRAME_M    0x20   //

#define LEGEND_MGR_FLAGS_CHIRALITY_MASK        0x03   // Mask that maps to the enum class.


/* Manuvr message defs. */
#define DIGITABULUM_MSG_IMU_LEGEND           0x0600 // No args? Asking for this legend. One arg: Legend provided.
#define DIGITABULUM_MSG_IMU_INIT             0x0604 //
#define DIGITABULUM_MSG_IMU_READ             0x0605 // Signal to read a given set of IMUs.
#define DIGITABULUM_MSG_IMU_MAP_STATE        0x0606
#define DIGITABULUM_MSG_IMU_QUAT_CRUNCH      0x0609 // The given IMU has samples to grind into a quat.
#define DIGITABULUM_MSG_IMU_TAP              0x060A // The given IMU experienced a tap.
#define DIGITABULUM_MSG_IMU_DOUBLE_TAP       0x060B // The given IMU experienced a double tap.


#define PREALLOCATED_IIU_MEASUREMENTS           180


#define AG_BASE_0_SIZE      10
#define AG_BASE_1_SIZE       5
#define AG_BASE_2_SIZE       9
#define AG_BASE_3_SIZE       8

#define M_BASE_0_SIZE        6
#define M_BASE_1_SIZE        6
#define M_BASE_2_SIZE        4

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
* The LegendManager is a system service that deals with interpreting the raw data generated by
*   Digitabulum's sensor package and passes them off to any number of instantiated ManuLegends.
*
* Like the CPLDDriver, this class implements BusOpCallback. IMU data is read directly
*   into this class.
*/
class LegendManager : public EventReceiver, public BusOpCallback {
  public:
    LegendManager(BusAdapter<SPIBusOp>*);
    ~LegendManager();

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


    uint32_t totalSamples();

    IIU* fetchIIU(uint8_t idx);  // TODO: Should be private.

    inline ManuLegend* getActiveLegend() {    return operating_legend;    }
    int8_t setLegend(ManuLegend*);

    /* Expose our idea about handedness to other modules. */
    inline Chirality getChirality() {  return (Chirality) _er_flag(LEGEND_MGR_FLAGS_CHIRALITY_MASK);  };
    inline bool chiralityKnown() {     return _er_flag(LEGEND_MGR_FLAGS_CHIRALITY_KNOWN);  };


    static LegendManager* getInstance();

    static InertialMeasurement* fetchMeasurement(uint8_t);
    static void reclaimMeasurement(InertialMeasurement*);
    static const char* chiralityString(Chirality);

    static Vector3<int16_t> reflection_mag;
    static Vector3<int16_t> reflection_acc;
    static Vector3<int16_t> reflection_gyr;


  protected:
    /* Overrides from EventReceiver */
    int8_t attached();


  private:
    CPLDDriver* _bus = nullptr;   // This is the gateway to the hardware.

    // TODO: These shouldn't be static.
    static IIU iius[LEGEND_DATASET_IIU_COUNT];  // This is the chirality-invarient list of IIUs.
    static InertialMeasurement __prealloc[PREALLOCATED_IIU_MEASUREMENTS];

    ManuLegend* operating_legend = nullptr;
    /* This is the dataset that we export. */
    uint8_t __dataset[LEGEND_MGR_MAX_DATASET_SIZE];


    // Used to direct data that we don't want in the legend until data-selection-by-NULL is implemented
    //   in the IIU class.
    uint32_t*       _ptr_sequence = nullptr;
    float*          _ptr_delta_t  = nullptr;

    ManuvrMsg event_legend_frame_ready;
    ManuvrMsg event_iiu_read;

    LinkedList<ManuLegend*> active_legends;  // We need to keep track of the legends we've released.

    // TODO: This will not scale. Think harder, Homer.
    uint8_t  last_imu_read        = 0;  // This is the IMU that is currently being operated on.

    int8_t send_map_event();

    int8_t init_iiu(uint8_t idx);
    int8_t read_identities();
    int8_t read_fifo_depth();

    int8_t refreshIMU();           // Calling causes the IMU to be read into its corresponding object.
    int8_t refreshIMU(uint8_t);    // Calling causes the IMU to be read into its corresponding object.

    int8_t reconfigure_data_map();    // Calling causes a pointer dance that reconfigures the data we send to the host.

    int set_chirality(Chirality);
    DigitPort  get_port_given_digit(Anatomical);
    Anatomical get_digit_given_port(DigitPort);

    /* Inlines for deriving address and IRQ bit offsets from index. */
    // Address of the inertial half of the LSM9DS1.
    inline uint8_t _intertial_addr(int idx) {   return ((idx % 17) + 0x00);   };
    // Address of the magnetic half of the LSM9DS1.
    inline uint8_t _magnetic_addr(int idx) {    return ((idx % 17) + 0x11);   };

    /**
    * Given an address, find the associated IIU.
    *
    * @param  test_addr The address to query.
    * @return A pointer to the IIU responsible for the given address.
    */
    inline IIU* fetch_iiu_by_bus_addr(uint8_t addr) {
      return (CPLD_REG_IMU_D5_D_M < addr) ? nullptr : fetchIIU(addr % CPLD_REG_IMU_D5_D_I);
    };

    /**
    * Given an address, find the associated IIU.
    *
    * @param  test_addr The address to query.
    * @return An index to the IIU responsible for the given address.
    */
    inline int8_t _iiu_idx_from_addr(uint8_t addr) {
      return (CPLD_REG_IMU_D5_D_M < addr) ? -1 : (addr % CPLD_REG_IMU_D5_D_I);
    };


    static LegendManager *INSTANCE;

    // These are preformed bus operations that address multiple IMUs...
    static SPIBusOp _preformed_read_a;
    static SPIBusOp _preformed_read_g;
    static SPIBusOp _preformed_read_m;
    static SPIBusOp _preformed_read_temp;
    static SPIBusOp _preformed_fifo_read;

    // Prealloc starvation counters...
    static uint32_t prealloc_starves;

    static uint32_t measurement_heap_instantiated;
    static uint32_t measurement_heap_freed;

    static uint32_t minimum_prealloc_level;

    static PriorityQueue<InertialMeasurement*>  preallocd_measurements;

    /* ---------------------- */
    /*    Register memory     */
    /* ---------------------- */

    /* These are giant strips of DMA-capable memory that are used for raw frame
         reads from the sensor package. Twice what we need for double-buffering. */
    static Vector3<int16_t> __frame_buf_a[2 * LEGEND_DATASET_IIU_COUNT];  // Inertial data
    static Vector3<int16_t> __frame_buf_g[2 * LEGEND_DATASET_IIU_COUNT];  // Inertial data
    static Vector3<int16_t> __frame_buf_m[2 * LEGEND_DATASET_IIU_COUNT];  // Mag data

    /* More large stretches of DMA memory. These are for IIU register definitions.
         Registers laid out this way cannot be multiply-accessed as more than single bytes
         by their respective IIU classes because the memory is not contiguous. */
    static int16_t __temperatures[LEGEND_DATASET_IIU_COUNT];
    static uint8_t __fifo_ctrl[LEGEND_DATASET_IIU_COUNT];
    static uint8_t __fifo_levels[LEGEND_DATASET_IIU_COUNT];  // The FIFO levels.
    static uint8_t __ag_status[LEGEND_DATASET_IIU_COUNT];

    /* Identity registers. */
    static uint8_t _imu_ids[2 * LEGEND_DATASET_IIU_COUNT];

    /* Accelerometer interrupt registers. */
    static uint8_t _reg_block_ag_0[LEGEND_DATASET_IIU_COUNT * AG_BASE_0_SIZE];

    /* Gyroscope control registers. */
    static uint8_t _reg_block_ag_1[LEGEND_DATASET_IIU_COUNT * AG_BASE_1_SIZE];

    /* Accelerometer control registers. */
    static uint8_t _reg_block_ag_2[LEGEND_DATASET_IIU_COUNT * AG_BASE_2_SIZE];

    /* Gyroscope interrupt registers. */
    static uint8_t _reg_block_ag_3[LEGEND_DATASET_IIU_COUNT * AG_BASE_3_SIZE];

    /* Magnetometer offset registers. */
    static uint8_t _reg_block_m_0[LEGEND_DATASET_IIU_COUNT * M_BASE_0_SIZE];

    /* Magnetometer control registers. */
    static uint8_t _reg_block_m_1[LEGEND_DATASET_IIU_COUNT * M_BASE_1_SIZE];

    /* Magnetometer interrupt registers. */
    static uint8_t _reg_block_m_2[LEGEND_DATASET_IIU_COUNT * M_BASE_2_SIZE];
    /* ---------------------- */
    /* End of register memory */
    /* ---------------------- */
};

#endif  // __DIGITABULUM_MANU_MGR_H_
